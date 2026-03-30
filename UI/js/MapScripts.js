import { data, otherAircraft, updatingWpAltitudes, setUpdatingWpAltitudes, DestinationCoordinates } from './CommScripts.js';

// ─── Map initialisation ───────────────────────────────────────────────────────
// pixelRatio: 1 renders at CSS pixel scale so the OS compositor handles
// upscaling — matching OpenLayers behaviour on high-DPI / OS-scaled displays.

var MAP_STYLES = {
    'dark':    'https://basemaps.cartocdn.com/gl/dark-matter-gl-style/style.json',
    'liberty': 'https://tiles.openfreemap.org/styles/liberty',
};

function getInitialMapStyle() {
    return MAP_STYLES[localStorage.getItem('ui_map_style')] || MAP_STYLES['liberty'];
}

export function setMapStyle(styleKey) {
    localStorage.setItem('ui_map_style', styleKey);
    map.setStyle(MAP_STYLES[styleKey] || MAP_STYLES['liberty']);
}

var map = new maplibregl.Map({
    container: 'map',
    style: getInitialMapStyle(),
    center: [-46.6652, -23.5467],
    zoom: 16,
    attributionControl: false,
});

// Silence "Image could not be loaded" noise from the style's sprite sheet.
// Providing placeholder images causes downstream null-expression warnings,
// so we filter the original message at the console level instead.
(function() {
    var _warn = console.warn.bind(console);
    console.warn = function() {
        if (typeof arguments[0] === 'string' &&
            arguments[0].indexOf('could not be loaded') !== -1) return;
        _warn.apply(console, arguments);
    };
}());

export let user_moved_map = false;
export function setUserMovedMap(val) { user_moved_map = val; }

window._gcssMap = map; // dev/screenshot access: _gcssMap.zoomTo(14), _gcssMap.setZoom(13), etc.

// ─── Secondary aircraft label overlay ─────────────────────────────────────────
// Labels live here (absolute inside #map) so they are never rotated with the marker.
var secLabelContainer = document.createElement('div');
secLabelContainer.style.cssText = 'position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none;overflow:hidden;z-index:2;';
document.getElementById('map').appendChild(secLabelContainer);

// ─── Secondary aircraft map objects ───────────────────────────────────────────
// Keyed by topic. Each value: { marker, markerEl, labelEl, iconHalf, addedToMap,
//                               courseSourceId, courseLayerId, pathSourceId, pathLayerId }
var secondaryAircraftObjects = {};

var onSecondaryAircraftClickCallback = null;
export function setOnSecondaryAircraftClick(fn) { onSecondaryAircraftClickCallback = fn; }

function sanitizeMapId(topic) {
    return 'sec_' + topic.replace(/[^a-zA-Z0-9]/g, '_');
}

function addSecondarySourcesAndLayers(entry, obj) {
    if (!map.getSource(obj.courseSourceId)) {
        map.addSource(obj.courseSourceId, { type: 'geojson', data: { type: 'Feature', geometry: null } });
    }
    if (!map.getLayer(obj.courseLayerId)) {
        map.addLayer({ id: obj.courseLayerId, type: 'line', source: obj.courseSourceId,
            paint: { 'line-color': entry.colour, 'line-width': (2 * window.devicePixelRatio) } });
    }
    if (!map.getSource(obj.pathSourceId)) {
        map.addSource(obj.pathSourceId, { type: 'geojson',
            data: { type: 'Feature', geometry: { type: 'LineString', coordinates: [] } } });
    }
    if (!map.getLayer(obj.pathLayerId)) {
        map.addLayer({ id: obj.pathLayerId, type: 'line', source: obj.pathSourceId,
            paint: { 'line-color': entry.colour, 'line-width': (2 * window.devicePixelRatio) } });
    }
}

function addSecondaryToMap(entry) {
    var safeId = sanitizeMapId(entry.topic);
    var imgWH  = 44 * window.devicePixelRatio;
    var obj = {
        courseSourceId: safeId + '_cs',
        courseLayerId:  safeId + '_cl',
        pathSourceId:   safeId + '_ps',
        pathLayerId:    safeId + '_pl',
        iconHalf:       imgWH / 2,
        addedToMap:     false,
    };

    // Marker element: aircraft image only — no label inside so it won't rotate with the icon.
    var markerEl = document.createElement('img');
    markerEl.src = 'img/aircraft.png';
    markerEl.style.cssText = 'width:' + imgWH + 'px;height:' + imgWH + 'px;cursor:pointer;display:block;' +
        'filter:invert(1) sepia(100%) hue-rotate(' + entry.hueRotate + 'deg);';

    // Label: lives in the screen-space overlay, positioned via map.project() each tick.
    // This prevents it from rotating with the aircraft icon.
    var labelEl = document.createElement('div');
    labelEl.style.cssText = 'position:absolute;display:none;transform:translateY(-50%);' +
        'background:rgba(0,0,0,0.72);color:#fff;white-space:nowrap;' +
        'font-size:' + (12 * window.devicePixelRatio) + 'px;' +
        'padding:' + (3 * window.devicePixelRatio) + 'px ' + (6 * window.devicePixelRatio) + 'px;' +
        'border-radius:' + (3 * window.devicePixelRatio) + 'px;' +
        'font-family:Ubuntu,sans-serif;' +
        'border-left:' + (3 * window.devicePixelRatio) + 'px solid ' + entry.colour + ';';
    secLabelContainer.appendChild(labelEl);

    // Marker is NOT added to the map yet — we add it on first valid GPS fix to
    // avoid the marker flashing at [0, 0] before any telemetry arrives.
    var marker = new maplibregl.Marker({ element: markerEl, rotationAlignment: 'map' });

    markerEl.addEventListener('click', function(e) {
        e.stopPropagation();
        if (onSecondaryAircraftClickCallback) onSecondaryAircraftClickCallback(entry.topic);
    });

    obj.marker   = marker;
    obj.markerEl = markerEl;
    obj.labelEl  = labelEl;
    secondaryAircraftObjects[entry.topic] = obj;

    if (map.isStyleLoaded()) addSecondarySourcesAndLayers(entry, obj);
}

function removeSecondaryFromMap(topic) {
    var obj = secondaryAircraftObjects[topic];
    if (!obj) return;
    if (obj.addedToMap) obj.marker.remove();
    obj.labelEl.remove();
    if (map.getLayer(obj.courseLayerId))   map.removeLayer(obj.courseLayerId);
    if (map.getSource(obj.courseSourceId)) map.removeSource(obj.courseSourceId);
    if (map.getLayer(obj.pathLayerId))     map.removeLayer(obj.pathLayerId);
    if (map.getSource(obj.pathSourceId))   map.removeSource(obj.pathSourceId);
    delete secondaryAircraftObjects[topic];
}

export function removeSecondaryAircraftFromMap(topic) {
    removeSecondaryFromMap(topic);
}

export function updateSecondaryAircraftOnMap() {
    var now = Date.now();

    // Remove map objects for topics that were deleted
    for (var t in secondaryAircraftObjects) {
        if (!otherAircraft[t]) removeSecondaryFromMap(t);
    }

    for (var topic in otherAircraft) {
        var entry = otherAircraft[topic];

        // Add map objects for newly registered topics
        if (!secondaryAircraftObjects[topic]) addSecondaryToMap(entry);

        if (entry.lastSeen === 0) continue; // no data yet — wait for first GPS fix

        var obj = secondaryAircraftObjects[topic];
        if (!obj) continue;

        // Dead-reckon position: project forward from last known fix using speed and course,
        // same approach as estimatePosition() for the primary aircraft.
        var latDeg, lonDeg;
        var elapsedSec = (now - entry.lastSeen) / 1000;
        if (entry.gsp > 0 && elapsedSec < 10) {
            var distM = (entry.gsp / 100) * elapsedSec; // metres since last fix
            var est   = DestinationCoordinates(entry.lat / 1e7, entry.lon / 1e7, entry.course, distM);
            latDeg = est.lat;
            lonDeg = est.lng;
        } else {
            latDeg = entry.lat / 1e7;
            lonDeg = entry.lon / 1e7;
        }

        var stale = (now - entry.lastSeen) > 10000;

        // Add marker to map on first valid fix (avoids the [0,0] flash before data arrives)
        if (!obj.addedToMap) {
            obj.marker.setLngLat([lonDeg, latDeg]).addTo(map);
            obj.addedToMap = true;
            obj.labelEl.style.display = '';
        } else {
            obj.marker.setLngLat([lonDeg, latDeg]);
        }

        obj.marker.setRotation(entry.course);
        obj.markerEl.style.opacity = stale ? '0.5' : '1';

        // Label: position in screen space via map.project() so it is never rotated.
        var px = map.project([lonDeg, latDeg]);
        obj.labelEl.style.left    = Math.round(px.x + obj.iconHalf + 4) + 'px';
        obj.labelEl.style.top     = Math.round(px.y) + 'px';
        obj.labelEl.style.opacity = stale ? '0.5' : '1';

        var altM    = (entry.alt / 100).toFixed(0);
        var spdKmh  = (entry.gsp / 27.78).toFixed(1);
        var climb   = entry.vsp > 20 ? '\u2191' : entry.vsp < -20 ? '\u2193' : '\u2014';
        var csLabel = entry.callsign || entry.topic.split('/').pop();
        obj.labelEl.textContent = csLabel + '  ' + altM + 'm  ' + spdKmh + '\u202fkm/h  ' + climb;

        // Course line — 1 minute of flight at current ground speed (same as primary)
        var courseSrc = map.getSource(obj.courseSourceId);
        if (courseSrc) {
            if (entry.gsp > 0) {
                var courseDistM = (entry.gsp / 100) * 60;
                var end = DestinationCoordinates(latDeg, lonDeg, entry.course, courseDistM);
                courseSrc.setData({ type: 'Feature', geometry: {
                    type: 'LineString',
                    coordinates: [[lonDeg, latDeg], [end.lng, end.lat]]
                }});
            } else {
                courseSrc.setData({ type: 'Feature', geometry: null });
            }
            map.setPaintProperty(obj.courseLayerId, 'line-opacity', stale ? 0.5 : 1.0);
        }

        // Flight path
        var pathSrc = map.getSource(obj.pathSourceId);
        if (pathSrc && entry.flightPath.length >= 2) {
            pathSrc.setData({ type: 'Feature', geometry: {
                type: 'LineString', coordinates: entry.flightPath
            }});
            map.setPaintProperty(obj.pathLayerId, 'line-opacity', stale ? 0.5 : 1.0);
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────

map.addControl(new maplibregl.NavigationControl({ showZoom: false, showCompass: true }), 'top-right');

var CenterOnAircraftControl = {
    onAdd: function() {
        this._btn = document.createElement('button');
        this._btn.type = 'button';
        this._btn.title = 'Center on aircraft';
        this._btn.className = 'maplibregl-ctrl-icon maplibregl-ctrl-center';
        this._btn.innerHTML = '<svg viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">' +
            '<circle cx="12" cy="12" r="7" fill="none" stroke="currentColor" stroke-width="2"/>' +
            '<circle cx="12" cy="12" r="2" fill="currentColor"/>' +
            '<line x1="12" y1="2" x2="12" y2="7" stroke="currentColor" stroke-width="2"/>' +
            '<line x1="12" y1="17" x2="12" y2="22" stroke="currentColor" stroke-width="2"/>' +
            '<line x1="2" y1="12" x2="7" y2="12" stroke="currentColor" stroke-width="2"/>' +
            '<line x1="17" y1="12" x2="22" y2="12" stroke="currentColor" stroke-width="2"/>' +
            '</svg>';
        this._btn.addEventListener('click', function() {
            user_moved_map = false;
            centerMap(data);
        });
        this._container = document.createElement('div');
        this._container.className = 'maplibregl-ctrl maplibregl-ctrl-group';
        this._container.appendChild(this._btn);
        return this._container;
    },
    onRemove: function() {
        this._container.parentNode.removeChild(this._container);
    }
};
map.addControl(CenterOnAircraftControl, 'top-right');

document.documentElement.style.setProperty('--compass-size', (44 * window.devicePixelRatio) + 'px');

map.on('drag', function() {
    user_moved_map = true;
});

var currZoom = map.getZoom();
map.on('zoomend', function() {
    var newZoom = map.getZoom();
    if (currZoom !== newZoom) {
        user_moved_map = true;
        currZoom = newZoom;
    }
});

// GeoJSON sources and line layers are added once the style finishes loading.
// Draw functions guard against being called before sources exist.
map.on('style.load', function() {
    map.resize();

    // Flight path (orange)
    map.addSource('flight-path-source', {
        type: 'geojson',
        data: { type: 'Feature', geometry: { type: 'LineString', coordinates: [] } }
    });
    map.addLayer({
        id: 'flight-path-layer',
        type: 'line',
        source: 'flight-path-source',
        paint: { 'line-color': '#F96', 'line-width': (3 * window.devicePixelRatio) }
    });

    // Course line (green)
    map.addSource('course-source', {
        type: 'geojson',
        data: { type: 'Feature', geometry: null }
    });
    map.addLayer({
        id: 'course-layer',
        type: 'line',
        source: 'course-source',
        paint: { 'line-color': '#9C6', 'line-width': (3 * window.devicePixelRatio) }
    });

    // Mission leg lines (blue)
    map.addSource('mission-lines-source', {
        type: 'geojson',
        data: { type: 'FeatureCollection', features: [] }
    });
    map.addLayer({
        id: 'mission-lines-layer',
        type: 'line',
        source: 'mission-lines-source',
        paint: { 'line-color': '#69F', 'line-width': (3 * window.devicePixelRatio) }
    });

    // Re-add secondary aircraft sources/layers (cleared when style reloads)
    for (var secTopic in secondaryAircraftObjects) {
        var secEntry = otherAircraft[secTopic];
        if (secEntry) addSecondarySourcesAndLayers(secEntry, secondaryAircraftObjects[secTopic]);
    }
});

// ─── Aircraft marker ──────────────────────────────────────────────────────────

var aircraftEl = document.createElement('img');

aircraftEl.src = 'img/aircraft.png';
aircraftEl.style.width  = (44 * window.devicePixelRatio) + 'px';
aircraftEl.style.height = (44 * window.devicePixelRatio) + 'px';

var aircraftMarker = new maplibregl.Marker({ element: aircraftEl, rotationAlignment: 'map' })
    .setLngLat([-46.6652, -23.5467])
    .addTo(map);

export function drawAircraftOnMap(inputData) {
    aircraftMarker.setLngLat([inputData.estimations.gpsLongitude, inputData.estimations.gpsLatitude]);
    aircraftMarker.setRotation(inputData.estimations.heading);
}

// ─── Course line ──────────────────────────────────────────────────────────────

export function drawCourseLineOnMap(inputData) {
    var src = map.getSource('course-source');
    if (!src) return;

    if (inputData.estimations.groundSpeed > 0) {
        var distance = inputData.estimations.groundSpeed / 1.667;
        var end = DestinationCoordinates(
            inputData.estimations.gpsLatitude,
            inputData.estimations.gpsLongitude,
            inputData.estimations.gpsGroundCourse,
            distance
        );
        src.setData({
            type: 'Feature',
            geometry: {
                type: 'LineString',
                coordinates: [
                    [inputData.estimations.gpsLongitude, inputData.estimations.gpsLatitude],
                    [end.lng, end.lat],
                ]
            }
        });
    } else {
        src.setData({ type: 'Feature', geometry: null });
    }
}

// ─── Flight path ──────────────────────────────────────────────────────────────

export function drawAircraftPathOnMap(inputData) {
    var src = map.getSource('flight-path-source');
    if (!src) return;

    if (inputData.currentFlightWaypoints.length < 2) {
        src.setData({ type: 'Feature', geometry: { type: 'LineString', coordinates: [] } });
        return;
    }

    var coords = inputData.currentFlightWaypoints.map(function(wp) {
        return [wp.wpLongitude, wp.wpLatitude];
    });
    coords.push([inputData.gpsLongitude, inputData.gpsLatitude]);

    src.setData({
        type: 'Feature',
        geometry: { type: 'LineString', coordinates: coords }
    });
}

// ─── Mission waypoints and leg lines ─────────────────────────────────────────

var waypointMarkers = [];
var onWaypointClickCallback = null;
export function setOnWaypointClick(fn) { onWaypointClickCallback = fn; }

export function drawMissionOnMap(inputData) {
    waypointMarkers.forEach(function(m) { m.remove(); });
    waypointMarkers = [];

    var src = map.getSource('mission-lines-source');
    if (src) src.setData({ type: 'FeatureCollection', features: [] });

    if (inputData.waypointCount === 0 || inputData.currentMissionWaypoints.length === 0)
        return;

    if (inputData.waypointCount !== (inputData.currentMissionWaypoints.length - 1)) {
        console.log('Waypoint count (' + inputData.waypointCount + ') differs from mission WP count (' + inputData.currentMissionWaypoints.length + ').');
        return;
    }

    var lineFeatures = [];
    var previousWp;

    for (var i = 1; i <= inputData.waypointCount; i++) {
        var wp = inputData.currentMissionWaypoints[i];
        if (wp === undefined) continue;

        if (wp.wpAction === 1) {
            var pinSrc;
            if (inputData.currentWaypointNumber === wp.waypointNumber)
                pinSrc = 'img/map_pin_green.png';
            else if (inputData.currentWaypointNumber < wp.waypointNumber)
                pinSrc = 'img/map_pin_blue.png';
            else
                pinSrc = 'img/map_pin_gray.png';

            var elWidth  = 36 * window.devicePixelRatio;
            var elHeight = 50 * window.devicePixelRatio;
            var elFontSize = 14 * window.devicePixelRatio;

            var el = document.createElement('div');
            el.style.cssText = 'width:' + elWidth + 'px;height:' + elHeight + 'px;cursor:pointer;';

            var pin = document.createElement('img');
            pin.src = pinSrc;
            pin.style.cssText = 'width:' + elWidth + 'px;height:' + elHeight + 'px;';

            var numLabel = document.createElement('div');
            numLabel.textContent = wp.waypointNumber;
            numLabel.style.cssText = 'position:absolute;top:' + (12 * window.devicePixelRatio) + 'px;left:0;width:' + elWidth + 'px;text-align:center;' +
                'color:#000;font-size:' + elFontSize + 'px;font-family:Ubuntu,sans-serif;font-weight:bold;pointer-events:none;';

            el.appendChild(pin);
            el.appendChild(numLabel);

            if (inputData.isCurrentMissionElevationSet && wp.elevation !== undefined) {
                var wpAlt  = wp.wpAltitude / 100;
                var grAlt  = (inputData.homeAltitudeSL + wpAlt) - wp.elevation;
                var altColor = grAlt > 5 ? '#fff' : '#F33'; // Red if within 5m of ground or below
                var infoDiv = document.createElement('div');
                infoDiv.style.cssText = 'position:absolute;left:' + (40 * window.devicePixelRatio) + 'px;top:0;background:rgba(0,0,0,0.72);' +
                    'color:' + altColor + ';font-size:' + (13 * window.devicePixelRatio) + 'px;white-space:nowrap;padding:' + (3 * window.devicePixelRatio) + 'px ' + (6 * window.devicePixelRatio) + 'px;border-radius:' + (3 * window.devicePixelRatio) + 'px;' +
                    'font-family:Ubuntu,sans-serif;line-height:1.6;pointer-events:none;';
                infoDiv.innerHTML = 'WP alt: ' + wpAlt.toFixed(0)        + ' m<br>' +
                                    'ABS alt: ' + grAlt.toFixed(0)        + ' m';
                el.appendChild(infoDiv);
            }

            (function(wpNumber) {
                el.addEventListener('click', function(e) {
                    e.stopPropagation();
                    if (onWaypointClickCallback) onWaypointClickCallback(wpNumber);
                });
            })(wp.waypointNumber);

            var marker = new maplibregl.Marker({ element: el, anchor: 'bottom' })
                .setLngLat([wp.wpLongitude, wp.wpLatitude])
                .addTo(map);
            waypointMarkers.push(marker);

            if (previousWp) {
                lineFeatures.push({
                    type: 'Feature',
                    geometry: {
                        type: 'LineString',
                        coordinates: [
                            [previousWp.wpLongitude, previousWp.wpLatitude],
                            [wp.wpLongitude, wp.wpLatitude],
                        ]
                    }
                });
            }
            previousWp = wp;

        } else if (wp.wpAction === 4 && inputData.homeLatitude !== 0 && previousWp) {
            lineFeatures.push({
                type: 'Feature',
                geometry: {
                    type: 'LineString',
                    coordinates: [
                        [previousWp.wpLongitude, previousWp.wpLatitude],
                        [inputData.homeLongitude, inputData.homeLatitude],
                    ]
                }
            });
            previousWp = wp;
        }
    }

    if (src && lineFeatures.length > 0) {
        src.setData({ type: 'FeatureCollection', features: lineFeatures });
    }
}

// ─── Home marker ──────────────────────────────────────────────────────────────

var homeMarker = null;

export function drawHomeOnMap(inputData) {
    if (homeMarker) {
        homeMarker.remove();
        homeMarker = null;
    }

    if (inputData.homeLongitude === 0 && inputData.homeLatitude === 0)
        return;

    var el = document.createElement('div');
    el.style.cssText = 'display:flex;align-items:center;';

    var imgWH = 44 * window.devicePixelRatio;

    var img = document.createElement('img');
    img.src = 'img/home.png';
    img.style.cssText = 'width:' + imgWH + 'px;height:' + imgWH + 'px;';

    var label = document.createElement('div');
    label.style.cssText = 'position:absolute;left:' + ((44 + 4) * window.devicePixelRatio) + 'px;background:rgba(0,0,0,0.72);color:#fff;' +
        'font-size:' + (13 * window.devicePixelRatio) + 'px;white-space:nowrap;padding:' + (3 * window.devicePixelRatio) + 'px ' + (6 * window.devicePixelRatio) + 'px;border-radius:' + (3 * window.devicePixelRatio) + 'px;font-family:Ubuntu,sans-serif;';
    label.textContent = 'Alt SL: ' + inputData.homeAltitudeSL.toFixed(0) + ' m';

    el.appendChild(img);
    el.appendChild(label);

    homeMarker = new maplibregl.Marker({ element: el, anchor: 'center' })
        .setLngLat([inputData.homeLongitude, inputData.homeLatitude])
        .addTo(map);
}

// ─── User location marker ─────────────────────────────────────────────────────

export let hasUserLocation = false;
var userMarker = null;

export function drawUserOnMap(inputData) {
    if (!hasUserLocation) return;

    if (!userMarker) {
        var el = document.createElement('img');
        var imgWH = 56 * window.devicePixelRatio;

        el.src = 'img/user.png';
        el.style.width  = imgWH + 'px';
        el.style.height = imgWH + 'px';

        userMarker = new maplibregl.Marker({ element: el, rotationAlignment: 'map' })
            .setLngLat([inputData.userLongitude, inputData.userLatitude])
            .addTo(map);
    } else {
        userMarker.setLngLat([inputData.userLongitude, inputData.userLatitude]);
    }

    if (inputData.userHeading !== null) {
        userMarker.setRotation(inputData.userHeading);
    }
}

// ─── Compass heading (DeviceOrientationEvent) ─────────────────────────────────
// When active, compass heading takes priority over GPS movement heading so the
// marker rotates even when the user is standing still.

var compassActive = false;

function handleOrientationEvent(event) {
    var heading = null;
    if (typeof event.webkitCompassHeading === 'number' && !isNaN(event.webkitCompassHeading)) {
        // iOS: direct magnetic compass heading (0 = N, clockwise)
        heading = event.webkitCompassHeading;
    } else if (event.absolute === true && typeof event.alpha === 'number' && event.alpha !== null) {
        // Android / Chrome with absolute orientation: alpha is CCW from north → convert to CW
        heading = (360 - event.alpha) % 360;
    }
    if (heading !== null) {
        compassActive = true;
        data.userHeading = heading;
    }
}

// Export so PageScripts can call it from a user gesture (required on iOS 13+).
// On non-iOS devices this can be called without a gesture; it returns a resolved Promise.
export function startOrientationTracking() {
    if (typeof DeviceOrientationEvent === 'undefined') {
        return Promise.resolve(false);
    }

    if (typeof DeviceOrientationEvent.requestPermission === 'function') {
        // iOS 13+ — permission must be requested from a user gesture
        return DeviceOrientationEvent.requestPermission().then(function(state) {
            if (state === 'granted') {
                window.addEventListener('deviceorientation', handleOrientationEvent);
                return true;
            }
            return false;
        }).catch(function() { return false; });
    }

    // Android / desktop — no permission needed
    // Prefer deviceorientationabsolute (guaranteed relative to magnetic north)
    if ('ondeviceorientationabsolute' in window) {
        window.addEventListener('deviceorientationabsolute', handleOrientationEvent);
    } else {
        window.addEventListener('deviceorientation', handleOrientationEvent);
    }
    return Promise.resolve(true);
}

// ─── Map centering ────────────────────────────────────────────────────────────

export function centerMap(inputData) {
    map.easeTo({
        center: [inputData.estimations.gpsLongitude, inputData.estimations.gpsLatitude],
        duration: 200,
    });
}

// ─── Geolocation ──────────────────────────────────────────────────────────────

var geoOptions = {
    enableHighAccuracy: true,
    maximumAge: 5000,
};

export function getUserLocation() {
    if (window.location.protocol === 'file:')
        return;

    if (navigator.geolocation) {
        navigator.geolocation.watchPosition(getUserPosition, showUserLocationError, geoOptions);
    } else {
        console.log('Geolocation is not supported by this browser.');
    }
}

function getUserPosition(position) {
    data.userLatitude    = position.coords.latitude;
    data.userLongitude   = position.coords.longitude;
    // Only use GPS movement heading if the compass sensor is not active —
    // GPS heading is null when stationary and is the wrong value for orientation.
    if (!compassActive) {
        data.userHeading = position.coords.heading;
    }
    data.userAltitudeSL  = position.coords.altitude;
    hasUserLocation = true;
}

function showUserLocationError(error) {
    console.log(error);
    switch (error.code) {
        case error.PERMISSION_DENIED:
            console.log('User denied the request for Geolocation.'); break;
        case error.POSITION_UNAVAILABLE:
            console.log('Location information is unavailable.'); break;
        case error.TIMEOUT:
            console.log('The request to get user location timed out.'); break;
        case error.UNKNOWN_ERROR:
            console.log('An unknown error occurred.'); break;
    }
}

// ─── Elevation / waypoint altitude API ───────────────────────────────────────

function updateElevationData(elevData) {
    var elevationProvider = localStorage.getItem('ui_elevation_provider');
    var latitude;
    var longitude;

    if (elevationProvider === 'OpenTopoData' || elevationProvider === 'OpenTopoDataDirect') {
        if (elevData.status !== 'OK')
            return;
    }

    for (var i = 0; i < elevData.results.length; i++) {
        if (elevationProvider === 'OpenElevation') {
            latitude  = elevData.results[i].latitude;
            longitude = elevData.results[i].longitude;
        } else if (elevationProvider === 'OpenTopoData' || elevationProvider === 'OpenTopoDataDirect') {
            latitude  = elevData.results[i].location.lat;
            longitude = elevData.results[i].location.lng;
        }

        for (var j = 0; j < data.currentMissionWaypoints.length; j++) {
            if (typeof data.currentMissionWaypoints[j] === 'undefined')
                continue;

            if (data.currentMissionWaypoints[j].wpLatitude  === latitude &&
                data.currentMissionWaypoints[j].wpLongitude === longitude) {
                data.currentMissionWaypoints[j].elevation = elevData.results[i].elevation;
            }
        }
    }
    data.isCurrentMissionElevationSet = true;
}

export function getMissionWaypointsAltitude() {
    var elevationProvider = localStorage.getItem('ui_elevation_provider');

    if (location.protocol !== 'https:')
        return;

    if (data.waypointCount === 0 || data.currentMissionWaypoints.length === 0)
        return;

    if (data.waypointCount !== (data.currentMissionWaypoints.length - 1))
        return;

    var arrWPs    = { locations: [] };
    var locations = '';
    var apiURL    = '';

    for (var i = 1; i < data.currentMissionWaypoints.length; i++) {
        if (data.currentMissionWaypoints[i].wpLatitude  === 0 &&
            data.currentMissionWaypoints[i].wpLongitude === 0)
            continue;

        if (elevationProvider === 'OpenElevation') {
            arrWPs.locations.push({
                latitude:  data.currentMissionWaypoints[i].wpLatitude,
                longitude: data.currentMissionWaypoints[i].wpLongitude,
            });
        } else if (elevationProvider === 'OpenTopoData' || elevationProvider === 'OpenTopoDataDirect') {
            if (locations.length > 0) locations += '|';
            locations += data.currentMissionWaypoints[i].wpLatitude + ',' +
                         data.currentMissionWaypoints[i].wpLongitude;
        }
    }

    var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        if (xmlhttp.readyState === XMLHttpRequest.DONE) {
            if (xmlhttp.status === 200) {
                var jsonResponse = JSON.parse(xmlhttp.responseText);
                updateElevationData(jsonResponse);
            } else {
                console.log('Error getting waypoint altitudes. Status: ' + xmlhttp.status);
                console.log('Response: ' + xmlhttp.responseText);
            }
            setUpdatingWpAltitudes(false);
        }
    };
    setUpdatingWpAltitudes(true);

    if (elevationProvider === 'OpenElevation') {
        apiURL = 'https://api.open-elevation.com/api/v1/lookup';
        xmlhttp.open('POST', 'proxy.php', true);
        xmlhttp.setRequestHeader('Content-Type', 'application/json;charset=UTF-8');
        xmlhttp.setRequestHeader('Accept', '*/*');
        xmlhttp.setRequestHeader('X-Proxy-Url', apiURL);
        xmlhttp.send(JSON.stringify(arrWPs));
    } else if (elevationProvider === 'OpenTopoData') {
        apiURL = 'https://api.opentopodata.org/v1/mapzen?locations=' + locations;
        xmlhttp.open('GET', 'proxy.php', true);
        xmlhttp.setRequestHeader('Content-Type', 'application/json;charset=UTF-8');
        xmlhttp.setRequestHeader('Accept', '*/*');
        xmlhttp.setRequestHeader('X-Proxy-Url', apiURL);
        xmlhttp.send();
    } else if (elevationProvider === 'OpenTopoDataDirect') {
        apiURL = 'https://fpvsampa.opentopodata.org/v1/mapzen?locations=' + locations;
        xmlhttp.open('GET', apiURL, true);
        xmlhttp.send();
    }
}
