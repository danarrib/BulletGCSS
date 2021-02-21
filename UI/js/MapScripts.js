// Setup Map
var Map = ol.Map;
var Overlay = ol.Overlay;
var View = ol.View;
var Feature = ol.Feature;
var Point = ol.geom.Point;
var TileJSON = ol.source.TileJSON;
var VectorSource = ol.source.Vector;
var TileLayer = ol.layer.Tile;
var VectorLayer = ol.layer.Vector;
var toStringHDMS = ol.coordinate.toStringHDMS;
var toLonLat = ol.proj.toLonLat;
var Icon = ol.style.Icon;
var Style = ol.style.Style;

var map = new ol.Map({
target: 'map',
layers: [
    new ol.layer.Tile({
    source: new ol.source.OSM()
    })
],
view: new ol.View({
    center: ol.proj.fromLonLat([-46.6652, -23.5467]),
    zoom: 16
})
});

var user_moved_map = false;
map.on('pointerdrag', function (event) {
    if(user_moved_map == false) {
        user_moved_map = true;
    }
});

var currZoom = map.getView().getZoom();
map.on('moveend', function(e) {
var newZoom = map.getView().getZoom();
    if (currZoom != newZoom) {
        user_moved_map = true;
        currZoom = newZoom;
    }
});

// Add Aircraft Map Stuff
var aircraftIconFeatures = [];

var aircraftIconStyle = new ol.style.Style({
    image: new ol.style.Icon({
        anchor: [0.5, 0.5],
        anchorXUnits: 'fraction',
        anchorYUnits: 'fraction',
        opacity: 1.0,
        src: 'img/aircraft.png', 
        scale: (0.04 * window.devicePixelRatio),
        rotateWithView: true
        })
});

var aircraftVectorSource = new ol.source.Vector({
    features: aircraftIconFeatures //add an array of features
});

var aircraftVectorLayer = new ol.layer.Vector({
    source: aircraftVectorSource,
    style: aircraftIconStyle
});


function drawAircraftOnMap(inputData)
{
    // Removing Previous Aircraft from the VectorSource
    aircraftVectorSource.clear();

    // Adding the updated aircraft position to the VectorSource
    var iconGeometry = new ol.geom.Point(
        ol.proj.transform([inputData.estimations.gpsLongitude, inputData.estimations.gpsLatitude], 'EPSG:4326','EPSG:3857')
    );
    
    var aircraftIconFeature = new ol.Feature({
        geometry: iconGeometry
    });

    // Clear previous aircraft and add new
    aircraftIconFeatures = [];
    aircraftIconFeatures.push(aircraftIconFeature);			
    aircraftVectorSource.addFeatures(aircraftIconFeatures);
    
    // Set reading
    aircraftIconStyle.getImage().setRotation(AngleToRadians(inputData.heading));
}

// Add User Map Stuff
var userIconFeatures = [];

var userIconStyle = new ol.style.Style({
    image: new ol.style.Icon({
        anchor: [0.5, 0.5],
        anchorXUnits: 'fraction',
        anchorYUnits: 'fraction',
        opacity: 1.0,
        src: 'img/user.png', 
        scale: (0.06 * window.devicePixelRatio),
        rotateWithView: true
        })
});

var userVectorSource = new ol.source.Vector({
    features: userIconFeatures //add an array of features
});

var userVectorLayer = new ol.layer.Vector({
    source: userVectorSource,
    style: userIconStyle
});


function drawUserOnMap(data)
{
    if(!hasUserLocation)
        return;

    // Removing Previous Aircraft from the VectorSource
    userVectorSource.clear();

    // Adding the updated aircraft position to the VectorSource
    var iconGeometry = new ol.geom.Point(
        ol.proj.transform([data.userLongitude, data.userLatitude], 'EPSG:4326','EPSG:3857')
    );
    
    var userIconFeature = new ol.Feature({
        geometry: iconGeometry
    });

    // Clear previous aircraft and add new
    userIconFeatures = [];
    userIconFeatures.push(userIconFeature);			
    userVectorSource.addFeatures(userIconFeatures);
    
    // Set reading
    userIconStyle.getImage().setRotation(AngleToRadians(data.userHeading));
}

// Add Home point on
var homeIconFeatures = [];

function fn_homeIconStyle(feature)
{
    var txtElevation = "Alt SL: " + data.homeAltitudeSL.toFixed(0) + " m";

    var homeAlt = parseInt(feature.get("name"));

    var homeIconStyle = new ol.style.Style({
        image: new ol.style.Icon({
            anchor: [0.5, 0.5],
            anchorXUnits: 'fraction',
            anchorYUnits: 'fraction',
            opacity: 1.0,
            src: 'img/home.png', 
            scale: (0.04 * window.devicePixelRatio),
            rotateWithView: false,
            })
    });

    var home_textStyle = new ol.style.Style({
        text: new ol.style.Text({
			font: (14 * window.devicePixelRatio) + 'px Ubuntu,sans-serif',
			fill: new ol.style.Fill({ color: '#000' }),
			stroke: new ol.style.Stroke({
              color: '#fff', 
              width: (2 * window.devicePixelRatio)
            }),
            textAlign: 'left',
            // get the text from the feature - `this` is ol.Feature
            // and show only under certain resolution
            text: txtElevation,
            offsetX: (20 * window.devicePixelRatio),
            offsetY: (0 * window.devicePixelRatio)
		})
    });

    return [homeIconStyle, home_textStyle];
}



var homeVectorSource = new ol.source.Vector({
    features: homeIconFeatures //add an array of features
});

var homeVectorLayer = new ol.layer.Vector({
    source: homeVectorSource,
    style: fn_homeIconStyle,
});


function drawHomeOnMap(data)
{
    // Removing Previous Aircraft from the VectorSource
    homeVectorSource.clear();

    if(data.homeLongitude == 0 && data.homeLatitude == 0)
        return;

    // Adding the updated aircraft position to the VectorSource
    var iconGeometry = new ol.geom.Point(
        ol.proj.transform([data.homeLongitude, data.homeLatitude], 'EPSG:4326','EPSG:3857')
    );
    
    var homeIconFeature = new ol.Feature({
        geometry: iconGeometry
    });

    // Clear previous aircraft and add new
    homeIconFeatures = [];
    homeIconFeatures.push(homeIconFeature);			
    homeVectorSource.addFeatures(homeIconFeatures);
   
}

// Add Waypoints Map Stuff
var waypointIconFeatures = [];

var waypointVectorSource = new ol.source.Vector({
    features: waypointIconFeatures //add an array of features
});

function fn_waypointIconStyle(feature)
{
    var wp_number = parseInt(feature.get("name"));
    var wp_elevation = parseInt(feature.get("groundAltitude"));
    var wp_action = parseInt(feature.get("wpAction"));
    var wp_altitude = parseInt(feature.get("wpAltitude")) / 100;
    var hp_altitudeSL = data.homeAltitudeSL;
    var wp_groundAltitude = (hp_altitudeSL + wp_altitude) - wp_elevation;
    var txtElevation = "";

    if(wp_action == 1 && data.isCurrentMissionElevationSet)
    {
        txtElevation = "Elev: "  + wp_elevation.toFixed(0) + " m\n" +
        "WP Alt: " + wp_altitude.toFixed(0) + " m\n" +
        "GR Alt: " + wp_groundAltitude.toFixed(0) + " m";

    }

    var icon = 'img/map_pin_gray.png'
    if(data.currentWaypointNumber ==  wp_number)
        icon = 'img/map_pin_green.png'
    else if(data.currentWaypointNumber < wp_number)
        icon = 'img/map_pin_blue.png'

    var wp_iconStyle = new ol.style.Style({
        image: new ol.style.Icon({
            anchor: [0.5, 0.9],
            anchorXUnits: 'fraction',
            anchorYUnits: 'fraction',
            opacity: 1.0,
            src: icon, 
            scale: (0.10 * window.devicePixelRatio),
            rotateWithView: false,
        }),
        text: new ol.style.Text({
			font: (14 * window.devicePixelRatio) + 'px Ubuntu,sans-serif',
			fill: new ol.style.Fill({ color: '#000' }),
			stroke: new ol.style.Stroke({
              color: '#fff', 
              width: (2 * window.devicePixelRatio)
			}),
            // get the text from the feature - `this` is ol.Feature
            // and show only under certain resolution
            text: wp_number.toString(),
            offsetX: 0,
            offsetY: (-24 * window.devicePixelRatio)
		})
    });

    var wp_textStyle = new ol.style.Style({
        text: new ol.style.Text({
			font: (14 * window.devicePixelRatio) + 'px Ubuntu,sans-serif',
			fill: new ol.style.Fill({ color: '#000' }),
			stroke: new ol.style.Stroke({
              color: '#fff', 
              width: (2 * window.devicePixelRatio)
            }),
            textAlign: 'left',
            // get the text from the feature - `this` is ol.Feature
            // and show only under certain resolution
            text: txtElevation,
            offsetX: (20 * window.devicePixelRatio),
            offsetY: (-20 * window.devicePixelRatio)
		})
    });
    return [wp_iconStyle, wp_textStyle];
}

var waypointVectorLayer = new ol.layer.Vector({
    source: waypointVectorSource,
    //style: waypointIconStyle,
    style: fn_waypointIconStyle,
});

var linesFeatures = [];

var linesStyle = new ol.style.Style({
    stroke: new ol.style.Stroke({
        color: '#69F',
        width: 3 * window.devicePixelRatio
    })
});

var linesVectorSource = new ol.source.Vector({
    features: linesFeatures //add an array of features
});

var linesVectorLayer = new ol.layer.Vector({
    source: linesVectorSource,
    style: linesStyle,
});

function drawMissionOnMap(data) {
    waypointVectorSource.clear();
    waypointIconFeatures = [];

    linesVectorSource.clear();
    linesFeatures = [];

    if(data.waypointCount == 0 || data.currentMissionWaypoints.length == 0)
        return;

    if(data.waypointCount != (data.currentMissionWaypoints.length - 1))
    {
        console.log("Waypoint count (" + data.waypointCount + ") is different than current mission WP count (" + data.currentMissionWaypoints.length + ").");
        return;
    }

    // Starts at WP 1 since WP0 is home point
    var previousWp;
    for(var i = 1; i <= data.waypointCount; i++) {
        var wp = data.currentMissionWaypoints[i];
        
        // Action 0 = Regular Waypoint
        if(wp.wpAction == 1) {
            // Adding the updated aircraft position to the VectorSource
            var iconGeometry = new ol.geom.Point(
                ol.proj.transform([wp.wpLongitude, wp.wpLatitude], 'EPSG:4326','EPSG:3857')
            );
            
            var waypointIconFeature = new ol.Feature({
                geometry: iconGeometry,
                name: wp.waypointNumber,
                groundAltitude: wp.elevation,
                wpAltitude: wp.wpAltitude,
                wpAction: wp.wpAction,
            });

            waypointIconFeatures.push(waypointIconFeature);

            // From the second wp, draw a line to the previous one.
            if(i > 1) {
                var loc1 = ol.proj.fromLonLat([previousWp.wpLongitude, previousWp.wpLatitude]);
                var loc2 = ol.proj.fromLonLat([wp.wpLongitude, wp.wpLatitude]);
                
                iconGeometry = new ol.geom.LineString([loc1, loc2]);

                var lineFeature = new ol.Feature({
                    geometry: iconGeometry,
                    name: 'From ' + previousWp.waypointNumber + ' to ' + wp.waypointNumber,
                });

                linesFeatures.push(lineFeature);
            }

            previousWp = wp;
        }
        else if(wp.wpAction == 4 && data.homeLatitude != 0 && data.homeLatitude != 0) {
            var loc1 = ol.proj.fromLonLat([previousWp.wpLongitude, previousWp.wpLatitude]);
            var loc2 = ol.proj.fromLonLat([data.homeLongitude, data.homeLatitude]);
            
            iconGeometry = new ol.geom.LineString([loc1, loc2]);

            var lineFeature = new ol.Feature({
                geometry: iconGeometry,
                name: 'From ' + previousWp.waypointNumber + ' to ' + wp.waypointNumber,
            });

            linesFeatures.push(lineFeature);

            previousWp = wp;

        }

    }

    waypointVectorSource.addFeatures(waypointIconFeatures);
    linesVectorSource.addFeatures(linesFeatures);
    
}

var flightLineFeatures = [];

var flightLineStyle = new ol.style.Style({
    stroke: new ol.style.Stroke({
        color: '#F96',
        width: 3 * window.devicePixelRatio
    })
});

var flightLineVectorSource = new ol.source.Vector({
    features: flightLineFeatures //add an array of features
});

var flightLineVectorLayer = new ol.layer.Vector({
    source: flightLineVectorSource,
    style: flightLineStyle,
});

function drawAircraftPathOnMap(data)
{
    // Now, render the flight line
    flightLineVectorSource.clear();
    flightLineFeatures = [];
    if(data.currentFlightWaypoints.length > 1) {
        var loc1;
        var loc2;
        var flightLineFeature;

        for(i = 1; i < data.currentFlightWaypoints.length; i++)
        {
            loc1 = ol.proj.fromLonLat([data.currentFlightWaypoints[i-1].wpLongitude, data.currentFlightWaypoints[i-1].wpLatitude]);
            loc2 = ol.proj.fromLonLat([data.currentFlightWaypoints[i].wpLongitude, data.currentFlightWaypoints[i].wpLatitude]);
            
            iconGeometry = new ol.geom.LineString([loc1, loc2]);

            flightLineFeature = new ol.Feature({
                geometry: iconGeometry
            });

            flightLineFeatures.push(flightLineFeature);
        }

        // Add another one from the last waypoint to the aircraft
        loc1 = loc2;
        loc2 = ol.proj.fromLonLat([data.gpsLongitude, data.gpsLatitude]);
        
        iconGeometry = new ol.geom.LineString([loc1, loc2]);

        flightLineFeature = new ol.Feature({
            geometry: iconGeometry
        });

        flightLineFeatures.push(flightLineFeature);
    }
    

    flightLineVectorSource.addFeatures(flightLineFeatures);
}


var courseLineFeatures = [];

var courseLineStyle = new ol.style.Style({
    stroke: new ol.style.Stroke({
        color: '#9C6',
        width: 3 * window.devicePixelRatio
    })
});

var courseLineVectorSource = new ol.source.Vector({
    features: courseLineFeatures //add an array of features
});

var courseLineVectorLayer = new ol.layer.Vector({
    source: courseLineVectorSource,
    style: courseLineStyle,
});

function drawCourseLineOnMap(data)
{
    courseLineVectorSource.clear();
    courseLineFeatures = [];

    if(data.groundSpeed > 0) // Ground speed is cm/s
    {
        var distance = data.groundSpeed / 1.667;
        var lineEndCoordinates = DestinationCoordinates(data.gpsLatitude, data.gpsLongitude, data.gpsGroundCourse, distance);
        var loc1 = ol.proj.fromLonLat([lineEndCoordinates.lng, lineEndCoordinates.lat]);
        var loc2 = ol.proj.fromLonLat([data.gpsLongitude, data.gpsLatitude]);

        var iconGeometry = new ol.geom.LineString([loc1, loc2]);

        var courseLineFeature = new ol.Feature({
            geometry: iconGeometry
        });

        courseLineFeatures.push(courseLineFeature);
    }

    courseLineVectorSource.addFeatures(courseLineFeatures);
}

function DestinationCoordinates(lat1, lon1, brng, dist) {
    var a = 6378137,
        b = 6356752.3142,
        f = 1 / 298.257223563, // WGS-84 ellipsiod
        s = dist,
        alpha1 = AngleToRadians(brng),
        sinAlpha1 = Math.sin(alpha1),
        cosAlpha1 = Math.cos(alpha1),
        tanU1 = (1 - f) * Math.tan(AngleToRadians(lat1)),
        cosU1 = 1 / Math.sqrt((1 + tanU1 * tanU1)), sinU1 = tanU1 * cosU1,
        sigma1 = Math.atan2(tanU1, cosAlpha1),
        sinAlpha = cosU1 * sinAlpha1,
        cosSqAlpha = 1 - sinAlpha * sinAlpha,
        uSq = cosSqAlpha * (a * a - b * b) / (b * b),
        A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq))),
        B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq))),
        sigma = s / (b * A),
        sigmaP = 2 * Math.PI;
    while (Math.abs(sigma - sigmaP) > 1e-12) {
        var cos2SigmaM = Math.cos(2 * sigma1 + sigma),
            sinSigma = Math.sin(sigma),
            cosSigma = Math.cos(sigma),
            deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));
        sigmaP = sigma;
        sigma = s / (b * A) + deltaSigma;
    };
    var tmp = sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1,
        lat2 = Math.atan2(sinU1 * cosSigma + cosU1 * sinSigma * cosAlpha1, (1 - f) * Math.sqrt(sinAlpha * sinAlpha + tmp * tmp)),
        lambda = Math.atan2(sinSigma * sinAlpha1, cosU1 * cosSigma - sinU1 * sinSigma * cosAlpha1),
        C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha)),
        L = lambda - (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM))),
        revAz = Math.atan2(sinAlpha, -tmp); // final bearing
    return {
        lat: RadiansToAngle(lat2),
        lng: lon1 + RadiansToAngle(L),
      };
};

function centerMap(data) {
    // Center the map
    var oldCenter = map.getView().getCenter();
    map.getView().setCenter(
        ol.proj.fromLonLat([data.estimations.gpsLongitude, data.estimations.gpsLatitude])
        );
    var newCenter = map.getView().getCenter();
    map.getView().setCenter(oldCenter);
    map.getView().animate({center: newCenter});
}

var hasUserLocation = false;
var geoOptions = {
    enableHighAccuracy: true,
    maximumAge: 5000,
};
var watchPositionId;

function getUserLocation() {
    if(window.location.protocol === 'file:')
        return;
        
    if (navigator.geolocation) {
        watchPositionId = navigator.geolocation.watchPosition(getUserPosition, showUserLocationError, geoOptions);
    } else { 
      console.log("Geolocation is not supported by this browser.");
    }
  }
  
  function getUserPosition(position) {
    data.userLatitude = position.coords.latitude;
    data.userLongitude = position.coords.longitude;
    data.userHeading = position.coords.heading;
    data.userAltitudeSL = position.coords.altitude;
      
    hasUserLocation = true;
  }
  
  function showUserLocationError(error) {
    console.log(error);

    switch(error.code) {
      case error.PERMISSION_DENIED:
        console.log("User denied the request for Geolocation.");
        break;
      case error.POSITION_UNAVAILABLE:
        console.log("Location information is unavailable.");
        break;
      case error.TIMEOUT:
        console.log("The request to get user location timed out.");
        break;
      case error.UNKNOWN_ERROR:
        console.log("An unknown error occurred.");
        break;
    }
  }

function updateElevationData(elevData)
{
    var elevationProvider = localStorage.getItem("ui_elevation_provider");
    var latitude;
    var longitude;

    if(elevationProvider == "OpenTopoData" || elevationProvider == "OpenTopoDataDirect")
    {
        // Check if it's not a valid result
        if(elevData.status != "OK")
            return;
    }

    for(i=0; i<elevData.results.length; i++)
    {
        if(elevationProvider == "OpenElevation")
        {
            latitude = elevData.results[i].latitude;
            longitude = elevData.results[i].longitude;
        }
        else if(elevationProvider == "OpenTopoData" || elevationProvider == "OpenTopoDataDirect")
        {
            latitude = elevData.results[i].location.lat;
            longitude = elevData.results[i].location.lng;
        }

        // Find the WP with same coordinates
        for(j=0; j< data.currentMissionWaypoints.length; j++)
        {
            if(typeof data.currentMissionWaypoints[j] == "undefined")
                continue;

            if(data.currentMissionWaypoints[j].wpLatitude == latitude
                && data.currentMissionWaypoints[j].wpLongitude == longitude)
                {
                    data.currentMissionWaypoints[j].elevation = elevData.results[i].elevation;
                }
        }
    }
    data.isCurrentMissionElevationSet = true;
}

var updatingWpAltitudes = false;

function getMissionWaypointsAltitude()
{
    var elevationProvider = localStorage.getItem("ui_elevation_provider");

    // If not served by https, then it'll not work, just give up
    if(location.protocol !== "https:")
        return;

    // If mission is not loaded, there's no need to process
    if(data.waypointCount == 0 || data.currentMissionWaypoints.length == 0)
        return;

    if(data.waypointCount != (data.currentMissionWaypoints.length - 1))
        return;

    // First, build an object with all Waypoint coodinates
    var arrWPs = { locations: [] };
    var locations = "";
    var apiURL = "";

    for(i=1; i < data.currentMissionWaypoints.length; i++)
    {
        if(data.currentMissionWaypoints[i].wpLatitude == 0 && data.currentMissionWaypoints[i].wpLongitude == 0)
            continue;
            
        if(elevationProvider == "OpenElevation")
        {
            arrWPs.locations.push({
                latitude: data.currentMissionWaypoints[i].wpLatitude,
                longitude: data.currentMissionWaypoints[i].wpLongitude,
            });
        }
        else if(elevationProvider == "OpenTopoData" || elevationProvider == "OpenTopoDataDirect" )
        {
            if(locations.length > 0)
                locations = locations + "|"

            locations = locations + data.currentMissionWaypoints[i].wpLatitude + "," + data.currentMissionWaypoints[i].wpLongitude;
        }
    }

    // Now get the altitudes from API
    var xmlhttp = new XMLHttpRequest();

    xmlhttp.onreadystatechange = function() {
        if (xmlhttp.readyState == XMLHttpRequest.DONE) 
        {
            if (xmlhttp.status == 200) 
            {
                var jsonResponse = JSON.parse(xmlhttp.responseText);
                console.log("Elevation Reply:")
                console.log(jsonResponse);

                updateElevationData(jsonResponse);
            }
            else 
            {
                console.log("Error getting waypoint altitudes from API. Status: " + xmlhttp.status);
                console.log("Response text: " + xmlhttp.responseText);
            }
            updatingWpAltitudes = false;
        }
    };
    updatingWpAltitudes = true;

    if(elevationProvider == "OpenElevation")
    {
        apiURL = "https://api.open-elevation.com/api/v1/lookup";
        xmlhttp.open("POST", "proxy.php", true);
        xmlhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
        xmlhttp.setRequestHeader('Accept', '*/*');
        xmlhttp.setRequestHeader('X-Proxy-Url', apiURL);
        xmlhttp.send(JSON.stringify(arrWPs));
    }
    else if(elevationProvider == "OpenTopoData")
    {
        apiURL = "https://api.opentopodata.org/v1/mapzen?locations=" + locations;
        xmlhttp.open("GET", "proxy.php", true);
        xmlhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
        xmlhttp.setRequestHeader('Accept', '*/*');
        xmlhttp.setRequestHeader('X-Proxy-Url', apiURL);
        xmlhttp.send();
    }
    else if(elevationProvider == "OpenTopoDataDirect")
    {
        apiURL = "https://fpvsampa.opentopodata.org/v1/mapzen?locations=" + locations;
        xmlhttp.open("GET", apiURL, true);
        xmlhttp.send();
    }
}

// Adding Map Layers
map.addLayer(linesVectorLayer);
map.addLayer(flightLineVectorLayer);
map.addLayer(courseLineVectorLayer);
map.addLayer(waypointVectorLayer);
map.addLayer(userVectorLayer);
map.addLayer(homeVectorLayer);
map.addLayer(aircraftVectorLayer);
