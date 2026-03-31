// MissionPlannerScripts.js — Mission Planner for Bullet GCSS

import { data, mqttConnected, publishCommand, waitForCommandAck, clearMissionDownloadBuffer, getMissionDownloadBuffer } from './CommScripts.js';
import { hasUserLocation, queryElevations } from './MapScripts.js';

// ── State ─────────────────────────────────────────────────────────────────────

let plannerMap = null;
let plannerMapReady = false;
let plannerMapInitialized = false;
let plannerUpdateInterval = null;
let plannerUserMarker = null;

// Array of mission waypoints.
// Shape: { lat, lon, altM, action, speedMs, loiterSec }
let plannedMission = [];

// MapLibre Marker instances, parallel to plannedMission.
let wpMarkers = [];

let routeSourceAdded = false;
let modalWpIndex = -1;
let uploadAborted = false;
let currentMissionName = null;
let missionDirty = false;

const STORAGE_KEY = 'gcss_saved_missions';

// ── Action code mappings ──────────────────────────────────────────────────────

const ACTION_CODE_TO_STRING = {
    1: 'Waypoint',
    3: 'PosHoldTime',
    4: 'RTH',
    5: 'SetPOI',
    6: 'Jump',
    7: 'SetHead',
    8: 'Land',
};
const ACTION_STRING_TO_CODE = Object.fromEntries(
    Object.entries(ACTION_CODE_TO_STRING).map(([k, v]) => [v, Number(k)])
);

const ACTION_COLORS = {
    1: '#4AF',  // Waypoint — light blue
    3: '#4FA',  // Loiter   — light green
    4: '#A4F',  // RTH      — light purple
    8: '#EE4',  // Land     — light yellow
};

function getActionColor(action) {
    return ACTION_COLORS[action] || '#4AF';
}

// ── Helpers ───────────────────────────────────────────────────────────────────

function haversineM(lat1, lon1, lat2, lon2) {
    var R = 6371000;
    var φ1 = lat1 * Math.PI / 180, φ2 = lat2 * Math.PI / 180;
    var Δφ = (lat2 - lat1) * Math.PI / 180;
    var Δλ = (lon2 - lon1) * Math.PI / 180;
    var a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
            Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function totalDistanceM() {
    var d = 0;
    for (var i = 1; i < plannedMission.length; i++) {
        d += haversineM(
            plannedMission[i - 1].lat, plannedMission[i - 1].lon,
            plannedMission[i].lat,     plannedMission[i].lon
        );
    }
    return d;
}

function distanceLabel(metres) {
    if (metres >= 1000) return (metres / 1000).toFixed(2) + ' km';
    return Math.round(metres) + ' m';
}

function getPlannerMapStyle() {
    var key = localStorage.getItem('ui_map_style') || 'liberty';
    var styles = {
        'dark':    'https://basemaps.cartocdn.com/gl/dark-matter-gl-style/style.json',
        'liberty': 'https://tiles.openfreemap.org/styles/liberty',
    };
    return styles[key] || styles['liberty'];
}

// ── Stats bar ─────────────────────────────────────────────────────────────────

function updateStats() {
    var n = plannedMission.length;
    var maxWp = data.maxWaypoints > 0 ? data.maxWaypoints : 15;
    var dist = n > 1 ? ' · ' + distanceLabel(totalDistanceM()) : '';
    var limitStr = n >= maxWp ? ' (limit reached)' : '';
    document.getElementById('mpStats').textContent =
        n + (n === 1 ? ' waypoint' : ' waypoints') + dist + limitStr;

    var uploadBtn = document.getElementById('mpBtUpload');
    if (uploadBtn) uploadBtn.disabled = (n === 0 || !mqttConnected);
}

function updateMissionName() {
    var name = currentMissionName || 'Untitled mission';
    if (missionDirty) name += ' *';
    document.getElementById('mpMissionName').textContent = name;
}

// ── Route line ────────────────────────────────────────────────────────────────

function updateRouteLine() {
    if (!plannerMapReady) return;
    var coords = plannedMission.map(function(wp) { return [wp.lon, wp.lat]; });
    var geojson = { type: 'Feature', geometry: { type: 'LineString', coordinates: coords } };
    if (routeSourceAdded) {
        plannerMap.getSource('mp-route').setData(geojson);
    } else if (coords.length >= 2) {
        plannerMap.addSource('mp-route', { type: 'geojson', data: geojson });
        plannerMap.addLayer({
            id: 'mp-route-line',
            type: 'line',
            source: 'mp-route',
            paint: {
                'line-color': '#4AF',
                'line-width': 3 * window.devicePixelRatio,
                'line-dasharray': [2, 1],
            },
        });
        routeSourceAdded = true;
    }
}

// ── WP markers ────────────────────────────────────────────────────────────────

function createMarkerElement(index, action) {
    var el = document.createElement('div');
    el.className = 'wp-marker';
    el.textContent = String(index + 1);
    el.style.cssText = [
        'width:8vmin', 'height:8vmin',
        'background:' + getActionColor(action),
        'border-radius:50%',
        'color:#000', 'font-weight:bold', 'display:flex', 'align-items:center',
        'justify-content:center', 'cursor:pointer', 'font-size:3.5vmin',
        'border:2px solid #fff', 'box-shadow:0 2px 6px rgba(0,0,0,0.5)',
        'user-select:none', '-webkit-user-select:none',
    ].join(';');
    return el;
}

function addWpMarker(index) {
    var wp = plannedMission[index];
    var el = createMarkerElement(index, wp.action);
    var marker = new maplibregl.Marker({ element: el, draggable: true })
        .setLngLat([wp.lon, wp.lat])
        .addTo(plannerMap);

    (function(i) {
        marker.on('drag', function() {
            var ll = marker.getLngLat();
            plannedMission[i].lat = ll.lat;
            plannedMission[i].lon = ll.lng;
            updateRouteLine();
        });
        marker.on('dragend', function() { missionDirty = true; updateStats(); updateMissionName(); });
        el.addEventListener('click', function(e) {
            e.stopPropagation();
            openWpModal(i);
        });
    })(index);

    wpMarkers.push(marker);
}

function clearAllMarkers() {
    wpMarkers.forEach(function(m) { m.remove(); });
    wpMarkers = [];
}

function rebuildAllMarkers() {
    clearAllMarkers();
    for (var i = 0; i < plannedMission.length; i++) {
        addWpMarker(i);
    }
}

function renumberMarkers() {
    for (var i = 0; i < wpMarkers.length; i++) {
        wpMarkers[i].getElement().textContent = String(i + 1);
    }
}

// ── User location marker ──────────────────────────────────────────────────────

function updatePlannerUserMarker() {
    if (!plannerMapReady) return;
    if (!hasUserLocation) return;
    if (!data.userLatitude && !data.userLongitude) return;

    if (!plannerUserMarker) {
        var imgWH = 36 * window.devicePixelRatio;
        var el = document.createElement('img');
        el.src = 'img/user.png';
        el.style.width  = imgWH + 'px';
        el.style.height = imgWH + 'px';
        plannerUserMarker = new maplibregl.Marker({ element: el, rotationAlignment: 'map' })
            .setLngLat([data.userLongitude, data.userLatitude])
            .addTo(plannerMap);
    } else {
        plannerUserMarker.setLngLat([data.userLongitude, data.userLatitude]);
    }

    if (data.userHeading !== null && data.userHeading !== undefined) {
        plannerUserMarker.setRotation(data.userHeading);
    }
}

// ── Terrain elevation ─────────────────────────────────────────────────────────

// Returns the ground elevation of the first WP (used as the home/base elevation).
function getPlannerBaseElevation() {
    if (plannedMission.length > 0 && plannedMission[0].groundElevM !== undefined)
        return plannedMission[0].groundElevM;
    return null;
}

// Update the elevation info row in the open modal without re-opening it.
function refreshModalElevation() {
    var elevRow = document.getElementById('wpModalElevRow');
    if (!elevRow) return;
    if (modalWpIndex < 0 || modalWpIndex >= plannedMission.length) {
        elevRow.style.display = 'none';
        return;
    }
    var wp = plannedMission[modalWpIndex];
    if (wp.groundElevM === undefined || wp.groundElevM === null) {
        elevRow.style.display = 'none';
        return;
    }
    var currentAltM = parseFloat(document.getElementById('wpModalAlt').value);
    if (isNaN(currentAltM)) currentAltM = wp.altM || 0;
    var base = getPlannerBaseElevation();
    var text = 'Terrain: ' + wp.groundElevM.toFixed(0) + '\u202fm';
    if (base !== null) {
        var aboveGround = currentAltM - (wp.groundElevM - base);
        text += '\u2002|\u2002~' + aboveGround.toFixed(0) + '\u202fm above terrain';
    }
    document.getElementById('wpModalElevText').textContent = text;
    elevRow.style.display = '';
}

// Fetch elevations for all planned waypoints in one batch request.
async function fetchAllPlannerElevations() {
    if (plannedMission.length === 0) return;
    var points = plannedMission.map(function(wp) { return { lat: wp.lat, lon: wp.lon }; });
    var results = await queryElevations(points);
    if (!results) return;
    for (var i = 0; i < results.length && i < plannedMission.length; i++)
        plannedMission[i].groundElevM = results[i].elevation;
    if (modalWpIndex >= 0 && modalWpIndex < plannedMission.length)
        refreshModalElevation();
}

// ── WP parameter modal ────────────────────────────────────────────────────────

function openWpModal(index) {
    modalWpIndex = index;
    var wp = plannedMission[index];

    document.getElementById('wpModalNum').textContent = String(index + 1);
    document.getElementById('wpModalAction').value = String(wp.action || 1);
    document.getElementById('wpModalAlt').value = String(wp.altM !== undefined ? wp.altM : 50);
    document.getElementById('wpModalSpeed').value = String(wp.speedMs !== undefined ? wp.speedMs : 0);
    document.getElementById('wpModalLoiter').value = String(wp.loiterSec !== undefined ? wp.loiterSec : 10);

    updateModalFieldVisibility();
    refreshModalElevation();

    var modal = document.getElementById('wpModal');
    modal.style.top = '50%';
    modal.style.left = '50%';
    modal.style.transform = 'translate(-50%,-50%)';

    modal.style.display = 'block';
}

function updateModalFieldVisibility() {
    var action = parseInt(document.getElementById('wpModalAction').value);
    document.getElementById('wpModalSpeedRow').style.display =
        (action === 1 || action === 3) ? '' : 'none';
    document.getElementById('wpModalLoiterRow').style.display =
        (action === 3) ? '' : 'none';
}

function saveAndCloseWpModal() {
    if (modalWpIndex < 0 || modalWpIndex >= plannedMission.length) {
        closeWpModal();
        return;
    }
    var idx = modalWpIndex;
    var wp = plannedMission[idx];
    var newAction = parseInt(document.getElementById('wpModalAction').value) || 1;

    // RTH must be the last waypoint — if set on a non-last WP, offer to delete what follows
    if (newAction === 4 && idx < plannedMission.length - 1) {
        var tail = plannedMission.length - 1 - idx;
        if (!confirm('RTH must be the last waypoint.\nDelete the ' + tail + ' waypoint(s) after this one?')) {
            // Revert the select back to the original action so the modal stays consistent
            document.getElementById('wpModalAction').value = String(wp.action || 1);
            updateModalFieldVisibility();
            return;
        }
        plannedMission.splice(idx + 1);
    }

    wp.action    = newAction;
    wp.altM      = parseFloat(document.getElementById('wpModalAlt').value) || 0;
    wp.speedMs   = parseFloat(document.getElementById('wpModalSpeed').value) || 0;
    wp.loiterSec = parseInt(document.getElementById('wpModalLoiter').value) || 10;
    closeWpModal();
    missionDirty = true;
    updateMissionName();
    rebuildAllMarkers();
    updateRouteLine();
    updateStats();
}

function closeWpModal() {
    document.getElementById('wpModal').style.display = 'none';
    modalWpIndex = -1;
}

function deleteWpFromModal() {
    if (modalWpIndex < 0 || modalWpIndex >= plannedMission.length) {
        closeWpModal();
        return;
    }
    if (!confirm('Delete waypoint ' + (modalWpIndex + 1) + '?')) return;
    plannedMission.splice(modalWpIndex, 1);
    closeWpModal();
    missionDirty = true;
    rebuildAllMarkers();
    updateRouteLine();
    updateStats();
    updateMissionName();
}

// ── Toast notification ────────────────────────────────────────────────────────

function showToast(msg) {
    var toast = document.getElementById('mpToast');
    if (!toast) return;
    toast.textContent = msg;
    toast.style.display = 'block';
    setTimeout(function() { toast.style.display = 'none'; }, 3000);
}

// ── Map initialisation ────────────────────────────────────────────────────────

function initPlannerMap() {
    if (plannerMapInitialized) return;
    plannerMapInitialized = true;

    plannerMap = new maplibregl.Map({
        container: 'mpMap',
        style: getPlannerMapStyle(),
        center: [data.gpsLongitude || 0, data.gpsLatitude || 0],
        zoom: 14,
        pixelRatio: 1,
    });

    plannerMap.addControl(new maplibregl.NavigationControl({ showCompass: false }), 'top-right');

    plannerMap.on('load', function() {
        plannerMapReady = true;
        rebuildAllMarkers();
        updateRouteLine();
        updatePlannerUserMarker();

        // Size navigation control buttons for touch usability
        var btnSizePx = Math.round(30 * window.devicePixelRatio) + 'px';
        var style = document.createElement('style');
        style.textContent = '#mpMap .maplibregl-ctrl button { width:' + btnSizePx + ' !important; height:' + btnSizePx + ' !important; }';
        document.head.appendChild(style);
    });

    // Clicking the map closes any open modal, or adds a new waypoint
    plannerMap.on('click', function(e) {
        if (document.getElementById('wpModal').style.display !== 'none') {
            saveAndCloseWpModal();
            return;
        }

        var maxWp = data.maxWaypoints > 0 ? data.maxWaypoints : 15;
        if (plannedMission.length >= maxWp) {
            showToast('Waypoint limit reached (' + maxWp + ')');
            return;
        }

        if (plannedMission.length > 0 && plannedMission[plannedMission.length - 1].action === 4) {
            showToast('RTH must be the last waypoint — cannot add after it');
            return;
        }

        var ll = e.lngLat;
        plannedMission.push({ lat: ll.lat, lon: ll.lng, altM: 50, action: 1, speedMs: 0, loiterSec: 10 });
        var newIdx = plannedMission.length - 1;
        missionDirty = true;
        addWpMarker(newIdx);
        updateRouteLine();
        updateStats();
        updateMissionName();
        openWpModal(newIdx);
        // Fetch terrain elevation for the new WP asynchronously
        (function(idx, lat, lon) {
            queryElevations([{lat: lat, lon: lon}]).then(function(results) {
                if (results && results.length > 0 && idx < plannedMission.length) {
                    plannedMission[idx].groundElevM = results[0].elevation;
                    if (modalWpIndex === idx) refreshModalElevation();
                }
            });
        })(newIdx, ll.lat, ll.lng);
    });
}

// ── Upload ────────────────────────────────────────────────────────────────────

async function uploadMission() {
    if (plannedMission.length === 0) { alert('No waypoints planned.'); return; }
    if (!mqttConnected) { alert('Not connected to MQTT broker.'); return; }
    if (data.fmWp === 1) {
        alert('Cannot upload: WP Mission mode is currently active on the aircraft.\nDeactivate it first.');
        return;
    }
    var maxWp = data.maxWaypoints > 0 ? data.maxWaypoints : 15;
    if (plannedMission.length > maxWp) {
        alert('Too many waypoints: ' + plannedMission.length + ' planned, max is ' + maxWp + '.');
        return;
    }

    uploadAborted = false;
    var total = plannedMission.length;
    var progressEl = document.getElementById('mpUploadProgress');
    var progressText = document.getElementById('mpUploadProgressText');
    progressEl.style.display = 'block';

    for (var i = 0; i < plannedMission.length; i++) {
        if (uploadAborted) break;
        var wp = plannedMission[i];
        var wpno = i + 1;
        var isLast = (i === plannedMission.length - 1);
        progressText.textContent = 'Uploading mission\u2026 WP ' + wpno + ' / ' + total;

        var altCm = Math.round((wp.altM || 0) * 100);
        var speedCms = Math.round((wp.speedMs || 0) * 100);
        var p1 = 0, p2 = 0;
        if (wp.action === 1) { p1 = speedCms; }
        else if (wp.action === 3) { p1 = wp.loiterSec || 10; p2 = speedCms; }

        var extraFields = {
            wpno: wpno,
            la: Math.round((wp.lat || 0) * 1e7),
            lo: Math.round((wp.lon || 0) * 1e7),
            al: altCm, ac: wp.action || 1,
            p1: p1, p2: p2, p3: 0,
            f: isLast ? 165 : 0,
        };

        var cid = await publishCommand('setwp', null, extraFields, 'WP ' + wpno + '/' + total);
        if (!cid) {
            progressEl.style.display = 'none';
            alert('Failed to send WP ' + wpno + ': could not publish command.');
            return;
        }
        try {
            await waitForCommandAck(cid, 15000);
        } catch (err) {
            if (!uploadAborted) {
                progressEl.style.display = 'none';
                alert('Upload failed at WP ' + wpno + ' / ' + total + '.\nReason: ' + err.message +
                      '\nThe aircraft\'s mission was not modified.');
            }
            return;
        }
    }

    progressEl.style.display = 'none';
    if (!uploadAborted) {
        progressText.textContent = 'Mission uploaded successfully!';
        progressEl.style.display = 'block';
        setTimeout(function() { progressEl.style.display = 'none'; }, 3000);
    }
}

// ── Download ──────────────────────────────────────────────────────────────────

async function downloadMission() {
    if (!mqttConnected) { alert('Not connected to MQTT broker.'); return; }

    var progressEl   = document.getElementById('mpUploadProgress');
    var progressText = document.getElementById('mpUploadProgressText');
    progressEl.style.display = 'block';
    progressText.textContent = 'Downloading mission\u2026';

    clearMissionDownloadBuffer();
    var cid = await publishCommand('getmission', null, {}, 'Download mission');
    if (!cid) {
        progressEl.style.display = 'none';
        alert('Failed to send getmission command.');
        return;
    }
    try {
        await waitForCommandAck(cid, 30000);
    } catch (err) {
        progressEl.style.display = 'none';
        alert('Download failed.\nReason: ' + err.message);
        return;
    }

    // WP 0 is the MSP home/RTH position, not a mission waypoint — skip it
    var buffer = getMissionDownloadBuffer().filter(function(wp) { return wp.dlwp !== 0; });
    if (buffer.length === 0) {
        progressEl.style.display = 'none';
        alert('No waypoints received from aircraft.');
        return;
    }

    // Convert dlwp buffer entries to plannedMission format
    var newMission = buffer.map(function(wp) {
        var entry = {
            lat:    wp.la / 1e7,
            lon:    wp.lo / 1e7,
            altM:   wp.al / 100,
            action: wp.ac,
            p1:     wp.p1,
            p2:     wp.p2,
            p3:     wp.p3,
        };
        // Reconstruct speed/loiter from p1/p2 depending on action
        if (wp.ac === 1) {
            entry.speedMs = wp.p1 / 100;
        } else if (wp.ac === 3) {
            entry.loiterSec = wp.p1;
            entry.speedMs   = wp.p2 / 100;
        }
        return entry;
    });

    loadMissionFromStorage(null, newMission);
    missionDirty = true;
    updateMissionName();

    progressEl.style.display = 'none';
    progressText.textContent = 'Mission downloaded: ' + newMission.length + ' waypoints';
    progressEl.style.display = 'block';
    setTimeout(function() { progressEl.style.display = 'none'; }, 3000);
}

// ── Local storage missions ────────────────────────────────────────────────────

function getSavedMissions() {
    try {
        return JSON.parse(localStorage.getItem(STORAGE_KEY) || '[]');
    } catch (e) {
        return [];
    }
}

function saveMissionToStorage(name) {
    var missions = getSavedMissions();
    var existing = missions.findIndex(function(m) { return m.name === name; });
    var entry = {
        name: name,
        savedAt: new Date().toISOString().slice(0, 10),
        waypoints: JSON.parse(JSON.stringify(plannedMission)),
    };
    if (existing >= 0) {
        missions[existing] = entry;
    } else {
        missions.push(entry);
    }
    localStorage.setItem(STORAGE_KEY, JSON.stringify(missions));
}

function deleteMissionFromStorage(name) {
    var missions = getSavedMissions().filter(function(m) { return m.name !== name; });
    localStorage.setItem(STORAGE_KEY, JSON.stringify(missions));
}

function loadMissionFromStorage(name, waypoints) {
    if (plannedMission.length > 0 && !confirm('Replace current mission?')) return;
    plannedMission = JSON.parse(JSON.stringify(waypoints));
    currentMissionName = name;
    missionDirty = false;
    rebuildAllMarkers();
    updateRouteLine();
    updateStats();
    updateMissionName();
    closeSavedMissionsPanel();
    if (plannedMission.length > 0 && plannerMapReady) {
        var lats = plannedMission.map(function(w) { return w.lat; });
        var lons = plannedMission.map(function(w) { return w.lon; });
        plannerMap.fitBounds(
            [[Math.min.apply(null, lons), Math.min.apply(null, lats)],
             [Math.max.apply(null, lons), Math.max.apply(null, lats)]],
            { padding: 60 }
        );
    }
    fetchAllPlannerElevations();
}

function saveMissionPrompt() {
    if (plannedMission.length === 0) { alert('No waypoints to save.'); return; }
    var name = prompt('Mission name:');
    if (!name || !name.trim()) return;
    name = name.trim();
    var missions = getSavedMissions();
    if (missions.some(function(m) { return m.name === name; })) {
        if (!confirm('A mission named "' + name + '" already exists. Overwrite?')) return;
    }
    saveMissionToStorage(name);
    currentMissionName = name;
    missionDirty = false;
    updateMissionName();
    showToast('Mission "' + name + '" saved.');
}

function showSavedMissionsPanel() {
    var missions = getSavedMissions();
    var list = document.getElementById('mpSavedMissionsList');
    var empty = document.getElementById('mpSavedMissionsEmpty');
    list.innerHTML = '';

    if (missions.length === 0) {
        empty.style.display = 'block';
    } else {
        empty.style.display = 'none';
        missions.forEach(function(m) {
            var row = document.createElement('div');
            row.style.cssText = 'display:flex;align-items:center;gap:2vmin;padding:2vmin 0;border-bottom:1px solid #333;';

            var info = document.createElement('div');
            info.style.cssText = 'flex:1;color:#fff;';
            info.innerHTML = '<div style="font-size:4vmin;font-weight:bold;">' + escapeHtml(m.name) + '</div>' +
                             '<div style="font-size:3vmin;color:#aaa;">' + m.savedAt + ' · ' +
                             m.waypoints.length + ' WP</div>';

            var loadBtn = document.createElement('input');
            loadBtn.type = 'button'; loadBtn.value = 'Load'; loadBtn.className = 'btRcCmd';
            loadBtn.addEventListener('click', (function(mName, waypoints) {
                return function() { loadMissionFromStorage(mName, waypoints); };
            })(m.name, m.waypoints));

            var delBtn = document.createElement('input');
            delBtn.type = 'button'; delBtn.value = 'Delete'; delBtn.className = 'btRcCmd';
            delBtn.style.background = '#722';
            delBtn.addEventListener('click', (function(name) {
                return function() {
                    if (!confirm('Delete saved mission "' + name + '"?')) return;
                    deleteMissionFromStorage(name);
                    showSavedMissionsPanel(); // refresh list
                };
            })(m.name));

            row.appendChild(info);
            row.appendChild(loadBtn);
            row.appendChild(delBtn);
            list.appendChild(row);
        });
    }

    document.getElementById('mpSavedMissionsPanel').style.display = 'block';
}

function closeSavedMissionsPanel() {
    document.getElementById('mpSavedMissionsPanel').style.display = 'none';
}

function escapeHtml(str) {
    return str.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

// ── Export / Import INAV JSON files ──────────────────────────────────────────

function exportMissionToFile() {
    if (plannedMission.length === 0) { alert('No waypoints to export.'); return; }
    var items = plannedMission.map(function(wp, i) {
        var isLast = (i === plannedMission.length - 1);
        var speedCms = Math.round((wp.speedMs || 0) * 100);
        var p1 = 0, p2 = 0;
        if (wp.action === 1) { p1 = speedCms; }
        else if (wp.action === 3) { p1 = wp.loiterSec || 10; p2 = speedCms; }
        return {
            type: ACTION_CODE_TO_STRING[wp.action] || 'Waypoint',
            lat: wp.lat, lon: wp.lon, alt: wp.altM,
            p1: p1, p2: p2, p3: 0,
            flag: isLast ? 165 : 0,
        };
    });
    var json = JSON.stringify({ version: 2, items: items }, null, 2);
    var blob = new Blob([json], { type: 'application/json' });
    var a = document.createElement('a');
    a.download = 'mission_' + new Date().toISOString().slice(0, 10) + '.json';
    a.href = URL.createObjectURL(blob);
    a.click();
}

function importMissionFromFile() {
    var input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.addEventListener('change', function() {
        if (!input.files || !input.files[0]) return;
        var reader = new FileReader();
        reader.onload = function(e) {
            try {
                var obj = JSON.parse(e.target.result);
                if (!obj.items || !Array.isArray(obj.items) || obj.items.length === 0)
                    throw new Error('No waypoints found in file');
                if (plannedMission.length > 0 && !confirm('Replace current mission with the imported file?'))
                    return;
                var loaded = [];
                for (var j = 0; j < obj.items.length; j++) {
                    var item = obj.items[j];
                    var action = ACTION_STRING_TO_CODE[item.type] || 1;
                    var speedMs = 0, loiterSec = 10;
                    if (action === 1) speedMs = (item.p1 || 0) / 100;
                    else if (action === 3) { loiterSec = item.p1 || 10; speedMs = (item.p2 || 0) / 100; }
                    loaded.push({ lat: item.lat, lon: item.lon, altM: item.alt,
                                  action: action, speedMs: speedMs, loiterSec: loiterSec });
                }
                plannedMission = loaded;
                currentMissionName = null;
                missionDirty = true;
                rebuildAllMarkers();
                updateRouteLine();
                updateStats();
                updateMissionName();
                if (plannedMission.length > 0 && plannerMapReady) {
                    var lats = plannedMission.map(function(w) { return w.lat; });
                    var lons = plannedMission.map(function(w) { return w.lon; });
                    plannerMap.fitBounds(
                        [[Math.min.apply(null, lons), Math.min.apply(null, lats)],
                         [Math.max.apply(null, lons), Math.max.apply(null, lats)]],
                        { padding: 60 }
                    );
                }
                fetchAllPlannerElevations();
            } catch (err) {
                alert('Failed to import mission file: ' + err.message);
            }
        };
        reader.readAsText(input.files[0]);
    });
    input.click();
}

// ── Clear mission ─────────────────────────────────────────────────────────────

function clearMission() {
    if (plannedMission.length === 0) return;
    if (!confirm('Clear all waypoints?')) return;
    plannedMission = [];
    currentMissionName = null;
    missionDirty = false;
    clearAllMarkers();
    updateRouteLine();
    updateStats();
    updateMissionName();
}

// ── Open / close planner view ─────────────────────────────────────────────────

function syncStatusIcons() {
    var connSrc = document.getElementById('connectionIcon');
    var cmdSrc  = document.getElementById('commandChannelIcon');
    if (connSrc) document.getElementById('mpConnectionIcon').src = connSrc.src;
    if (cmdSrc)  document.getElementById('mpCommandChannelIcon').src = cmdSrc.src;
    var dot = document.getElementById('mpWpValidDot');
    if (dot) {
        var valid = data.isWaypointMissionValid;
        dot.style.background = valid === 1 ? '#4c4' : '#c44';
        dot.title = valid === 1 ? 'Mission valid (FC reports OK)' : 'Mission not valid or not loaded on FC';
    }
}

export function openMissionPlanner() {
    document.getElementById('missionPlannerView').style.display = 'block';
    syncStatusIcons();
    if (!plannerMapInitialized) {
        requestAnimationFrame(function() { initPlannerMap(); });
    } else if (plannerMapReady) {
        plannerMap.resize();
        rebuildAllMarkers();
        updateRouteLine();
        updatePlannerUserMarker();
    }
    updateStats();
    updateMissionName();
    plannerUpdateInterval = setInterval(function() {
        updatePlannerUserMarker();
        updateStats();
        syncStatusIcons();
    }, 2000);
}

export function closeMissionPlanner() {
    document.getElementById('missionPlannerView').style.display = 'none';
    closeSavedMissionsPanel();
    closeWpModal();
    if (plannerUpdateInterval) { clearInterval(plannerUpdateInterval); plannerUpdateInterval = null; }
}

// ── Wire button handlers ──────────────────────────────────────────────────────

export function initMissionPlannerButtons() {
    document.getElementById('mpBtClose').addEventListener('click', closeMissionPlanner);
    document.getElementById('mpBtUpload').addEventListener('click', function() { uploadMission(); });
    document.getElementById('mpBtDownload').addEventListener('click', function() { downloadMission(); });
    document.getElementById('mpBtSave').addEventListener('click', saveMissionPrompt);
    document.getElementById('mpBtLoad').addEventListener('click', showSavedMissionsPanel);
    document.getElementById('mpBtExport').addEventListener('click', exportMissionToFile);
    document.getElementById('mpBtImport').addEventListener('click', importMissionFromFile);
    document.getElementById('mpBtClear').addEventListener('click', clearMission);
    document.getElementById('mpBtCancelUpload').addEventListener('click', function() {
        uploadAborted = true;
        document.getElementById('mpUploadProgress').style.display = 'none';
    });
    document.getElementById('mpBtCloseSavedList').addEventListener('click', closeSavedMissionsPanel);
    document.getElementById('wpModalAction').addEventListener('change', updateModalFieldVisibility);
    document.getElementById('wpModalAlt').addEventListener('input', refreshModalElevation);
    document.getElementById('wpModalClose').addEventListener('click', saveAndCloseWpModal);
    document.getElementById('wpModalDelete').addEventListener('click', deleteWpFromModal);
}
