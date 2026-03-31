// MissionPlannerScripts.js — Mission Planner for Bullet GCSS
// Owns all planned-mission state and UI logic. Lazy-initialises a second
// MapLibre GL JS map instance the first time the planner view is opened.

import { data, mqttConnected, publishCommand, waitForCommandAck } from './CommScripts.js';

// ── State ─────────────────────────────────────────────────────────────────────

let plannerMap = null;
let plannerMapReady = false;    // true after map 'load' event fires
let plannerMapInitialized = false; // true after initPlannerMap() called

// Array of mission waypoints.
// Shape: { lat, lon, altM, action, speedMs, loiterSec }
let plannedMission = [];

// MapLibre Marker instances, parallel to plannedMission.
let wpMarkers = [];

let routeSourceAdded = false;
let modalWpIndex = -1;   // which WP the modal is editing (-1 = none)
let uploadAborted = false;

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

// ── Helpers ───────────────────────────────────────────────────────────────────

function haversineM(lat1, lon1, lat2, lon2) {
    const R = 6371000;
    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;
    const Δφ = (lat2 - lat1) * Math.PI / 180;
    const Δλ = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(Δφ / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
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
    if (uploadBtn) {
        uploadBtn.disabled = (n === 0 || !mqttConnected);
    }
}

// ── Route line ────────────────────────────────────────────────────────────────

function updateRouteLine() {
    if (!plannerMapReady) return;
    var coords = plannedMission.map(function(wp) { return [wp.lon, wp.lat]; });
    var geojson = {
        type: 'Feature',
        geometry: { type: 'LineString', coordinates: coords }
    };
    if (routeSourceAdded) {
        plannerMap.getSource('mp-route').setData(geojson);
    } else if (coords.length >= 2) {
        plannerMap.addSource('mp-route', { type: 'geojson', data: geojson });
        plannerMap.addLayer({
            id: 'mp-route-line',
            type: 'line',
            source: 'mp-route',
            paint: { 'line-color': '#4AF', 'line-width': 2, 'line-dasharray': [2, 1] }
        });
        routeSourceAdded = true;
    }
}

// ── WP markers ────────────────────────────────────────────────────────────────

function createMarkerElement(index) {
    var el = document.createElement('div');
    el.className = 'wp-marker';
    el.textContent = String(index + 1);
    el.style.cssText = [
        'width:8vmin', 'height:8vmin', 'background:#4AF', 'border-radius:50%',
        'color:#000', 'font-weight:bold', 'display:flex', 'align-items:center',
        'justify-content:center', 'cursor:pointer', 'font-size:3.5vmin',
        'border:2px solid #fff', 'box-shadow:0 2px 6px rgba(0,0,0,0.5)',
        'user-select:none', '-webkit-user-select:none',
    ].join(';');
    return el;
}

function addWpMarker(index) {
    var el = createMarkerElement(index);
    var wp = plannedMission[index];
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
        marker.on('dragend', function() {
            updateStats();
        });
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

// After a WP is deleted, renumber all existing marker elements in place.
function renumberMarkers() {
    for (var i = 0; i < wpMarkers.length; i++) {
        wpMarkers[i].getElement().textContent = String(i + 1);
    }
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

    // Position modal near the marker, centred on screen if map not ready
    var modal = document.getElementById('wpModal');
    modal.style.top = '50%';
    modal.style.left = '50%';
    modal.style.transform = 'translate(-50%,-50%)';

    if (plannerMapReady) {
        var px = plannerMap.project([wp.lon, wp.lat]);
        var vw = window.innerWidth;
        var vh = window.innerHeight;
        var mw = Math.min(vw * 0.8, 400);
        var left = Math.min(Math.max(px.x - mw / 2, 10), vw - mw - 10);
        var top = px.y + 30;
        if (top + 300 > vh) top = px.y - 320;
        if (top < 10) top = 10;
        modal.style.top = top + 'px';
        modal.style.left = left + 'px';
        modal.style.transform = 'none';
    }

    document.getElementById('wpModalOverlay').style.display = 'block';
    document.getElementById('wpModal').style.display = 'block';
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
    var wp = plannedMission[modalWpIndex];
    wp.action    = parseInt(document.getElementById('wpModalAction').value) || 1;
    wp.altM      = parseFloat(document.getElementById('wpModalAlt').value) || 0;
    wp.speedMs   = parseFloat(document.getElementById('wpModalSpeed').value) || 0;
    wp.loiterSec = parseInt(document.getElementById('wpModalLoiter').value) || 10;
    closeWpModal();
}

function closeWpModal() {
    document.getElementById('wpModalOverlay').style.display = 'none';
    document.getElementById('wpModal').style.display = 'none';
    modalWpIndex = -1;
}

function deleteWpFromModal() {
    if (modalWpIndex < 0 || modalWpIndex >= plannedMission.length) {
        closeWpModal();
        return;
    }
    plannedMission.splice(modalWpIndex, 1);
    closeWpModal();
    rebuildAllMarkers();
    updateRouteLine();
    updateStats();
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
    });

    plannerMap.on('click', function(e) {
        // Ignore clicks if a modal is open
        if (document.getElementById('wpModal').style.display !== 'none') return;

        var maxWp = data.maxWaypoints > 0 ? data.maxWaypoints : 15;
        if (plannedMission.length >= maxWp) {
            showToast('Waypoint limit reached (' + maxWp + ')');
            return;
        }

        var ll = e.lngLat;
        plannedMission.push({
            lat: ll.lat, lon: ll.lng, altM: 50,
            action: 1, speedMs: 0, loiterSec: 10
        });
        var newIdx = plannedMission.length - 1;
        if (plannerMapReady) addWpMarker(newIdx);
        updateRouteLine();
        updateStats();
        openWpModal(newIdx);
    });
}

// ── Toast notification ────────────────────────────────────────────────────────

function showToast(msg) {
    var toast = document.getElementById('mpToast');
    if (!toast) return;
    toast.textContent = msg;
    toast.style.display = 'block';
    setTimeout(function() { toast.style.display = 'none'; }, 3000);
}

// ── Upload ────────────────────────────────────────────────────────────────────

async function uploadMission() {
    if (plannedMission.length === 0) {
        alert('No waypoints planned.');
        return;
    }
    if (!mqttConnected) {
        alert('Not connected to MQTT broker.');
        return;
    }
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
        if (wp.action === 1) {        // WAYPOINT: p1 = speed cm/s
            p1 = speedCms;
        } else if (wp.action === 3) { // POSHOLD_TIME: p1 = loiter seconds, p2 = speed cm/s
            p1 = wp.loiterSec || 10;
            p2 = speedCms;
        }

        var extraFields = {
            wpno: wpno,
            la: Math.round((wp.lat || 0) * 1e7),
            lo: Math.round((wp.lon || 0) * 1e7),
            al: altCm,
            ac: wp.action || 1,
            p1: p1,
            p2: p2,
            p3: 0,
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
                alert('Upload failed at WP ' + wpno + ' / ' + total + '.\nReason: ' + err.message + '\nThe aircraft\'s mission was not modified.');
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

// ── Save / Load INAV JSON ─────────────────────────────────────────────────────

function saveMissionToFile() {
    if (plannedMission.length === 0) { alert('No waypoints to save.'); return; }
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

function loadMissionFromFile() {
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
                if (plannedMission.length > 0 && !confirm('Replace current mission with the loaded file?'))
                    return;
                var loaded = [];
                for (var j = 0; j < obj.items.length; j++) {
                    var item = obj.items[j];
                    var action = ACTION_STRING_TO_CODE[item.type] || 1;
                    var speedMs = 0, loiterSec = 10;
                    if (action === 1) speedMs = (item.p1 || 0) / 100;
                    else if (action === 3) { loiterSec = item.p1 || 10; speedMs = (item.p2 || 0) / 100; }
                    loaded.push({
                        lat: item.lat, lon: item.lon, altM: item.alt,
                        action: action, speedMs: speedMs, loiterSec: loiterSec,
                    });
                }
                plannedMission = loaded;
                rebuildAllMarkers();
                updateRouteLine();
                updateStats();
                if (plannedMission.length > 0 && plannerMapReady) {
                    var lats = plannedMission.map(function(w) { return w.lat; });
                    var lons = plannedMission.map(function(w) { return w.lon; });
                    plannerMap.fitBounds(
                        [[Math.min.apply(null, lons), Math.min.apply(null, lats)],
                         [Math.max.apply(null, lons), Math.max.apply(null, lats)]],
                        { padding: 60 }
                    );
                }
            } catch (err) {
                alert('Failed to load mission file: ' + err.message);
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
    clearAllMarkers();
    updateRouteLine();
    updateStats();
}

// ── Open / close planner view ─────────────────────────────────────────────────

export function openMissionPlanner() {
    document.getElementById('missionPlannerView').style.display = 'block';
    if (!plannerMapInitialized) {
        // Map div must be visible before init so MapLibre can read its dimensions
        requestAnimationFrame(function() {
            initPlannerMap();
        });
    } else if (plannerMapReady) {
        plannerMap.resize();
        rebuildAllMarkers();
        updateRouteLine();
    }
    updateStats();
}

export function closeMissionPlanner() {
    document.getElementById('missionPlannerView').style.display = 'none';
}

// ── Wire button handlers ──────────────────────────────────────────────────────

export function initMissionPlannerButtons() {
    document.getElementById('mpBtClose').addEventListener('click', closeMissionPlanner);

    document.getElementById('mpBtUpload').addEventListener('click', function() {
        uploadMission();
    });
    document.getElementById('mpBtSave').addEventListener('click', saveMissionToFile);
    document.getElementById('mpBtLoad').addEventListener('click', loadMissionFromFile);
    document.getElementById('mpBtClear').addEventListener('click', clearMission);

    document.getElementById('mpBtCancelUpload').addEventListener('click', function() {
        uploadAborted = true;
        document.getElementById('mpUploadProgress').style.display = 'none';
    });

    document.getElementById('wpModalAction').addEventListener('change', updateModalFieldVisibility);
    document.getElementById('wpModalClose').addEventListener('click', saveAndCloseWpModal);
    document.getElementById('wpModalDelete').addEventListener('click', deleteWpFromModal);
    document.getElementById('wpModalOverlay').addEventListener('click', saveAndCloseWpModal);
}
