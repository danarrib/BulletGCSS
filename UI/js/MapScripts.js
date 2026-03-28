import { data, updatingWpAltitudes, setUpdatingWpAltitudes, DestinationCoordinates } from './CommScripts.js';

// ─── Map initialisation ───────────────────────────────────────────────────────
// pixelRatio: 1 renders at CSS pixel scale so the OS compositor handles
// upscaling — matching OpenLayers behaviour on high-DPI / OS-scaled displays.

var map = new maplibregl.Map({
    container: 'map',
    style: 'https://tiles.openfreemap.org/styles/liberty',
    center: [-46.6652, -23.5467],
    zoom: 16,
    attributionControl: false,
});

window._map = map; // Expose map for debugging

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

map.addControl(new maplibregl.NavigationControl({ showZoom: false, showCompass: true }), 'top-right');

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
map.on('load', function() {
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
});

// ─── Aircraft marker ──────────────────────────────────────────────────────────

var aircraftEl = document.createElement('img');
aircraftEl.src = 'img/aircraft.png';
aircraftEl.style.width  = (56 * window.devicePixelRatio) + 'px';
aircraftEl.style.height = (56 * window.devicePixelRatio) + 'px';

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
    data.userHeading     = position.coords.heading;
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
                console.log('Elevation Reply:');
                console.log(jsonResponse);
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
