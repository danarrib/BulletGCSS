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
        scale: (0.05 * window.devicePixelRatio),
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


function drawAircraftOnMap(data)
{
    // Removing Previous Aircraft from the VectorSource
    aircraftVectorSource.clear();

    // Adding the updated aircraft position to the VectorSource
    var iconGeometry = new ol.geom.Point(
        ol.proj.transform([data.gpsLongitude, data.gpsLatitude], 'EPSG:4326','EPSG:3857')
    );
    
    var aircraftIconFeature = new ol.Feature({
        geometry: iconGeometry
    });

    // Clear previous aircraft and add new
    aircraftIconFeatures = [];
    aircraftIconFeatures.push(aircraftIconFeature);			
    aircraftVectorSource.addFeatures(aircraftIconFeatures);
    
    // Set reading
    aircraftIconStyle.getImage().setRotation(AngleToRadians(data.heading));
}

// Add Aircraft Map Stuff
var userIconFeatures = [];

var userIconStyle = new ol.style.Style({
    image: new ol.style.Icon({
        anchor: [0.5, 0.5],
        anchorXUnits: 'fraction',
        anchorYUnits: 'fraction',
        opacity: 1.0,
        src: 'img/cellphone.png', 
        scale: (0.04 * window.devicePixelRatio),
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

var homeVectorSource = new ol.source.Vector({
    features: homeIconFeatures //add an array of features
});

var homeVectorLayer = new ol.layer.Vector({
    source: homeVectorSource,
    style: homeIconStyle
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

var waypointIconStyle = new ol.style.Style({
    image: new ol.style.Icon({
        anchor: [0.5, 0.9],
        anchorXUnits: 'fraction',
        anchorYUnits: 'fraction',
        opacity: 1.0,
        src: 'img/map_pin_blue.png', 
        scale: (0.10 * window.devicePixelRatio),
        rotateWithView: false,
    })
});

var waypointVectorSource = new ol.source.Vector({
    features: waypointIconFeatures //add an array of features
});

var waypointVectorLayer = new ol.layer.Vector({
    source: waypointVectorSource,
    style: waypointIconStyle,
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
    // First, add the waypoint to the Array if the aircraft is armed
    if(data.uavIsArmed)
    {
        var wpCount = data.currentFlightWaypoints.length;

        var waypoint = {
            wpLatitude: data.gpsLatitude,
            wpLongitude: data.gpsLongitude,
        };

        data.currentFlightWaypoints[wpCount] = waypoint;
    }

    // Now, render the flight line
    flightLineVectorSource.clear();
    flightLineFeatures = [];
    if(data.currentFlightWaypoints.length > 0) {
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

function centerMap(data) {
    // Center the map
    var oldCenter = map.getView().getCenter();
    map.getView().setCenter(
        ol.proj.fromLonLat([data.gpsLongitude, data.gpsLatitude])
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



// Adding Map Layers
map.addLayer(linesVectorLayer);
map.addLayer(flightLineVectorLayer);
map.addLayer(waypointVectorLayer);
map.addLayer(userVectorLayer);
map.addLayer(homeVectorLayer);
map.addLayer(aircraftVectorLayer);
