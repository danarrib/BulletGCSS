// Setup viewport
var windowOuterWidth = 0;

function UpdateViewPortSize(resize=true) {
    if(windowOuterWidth != window.outerWidth) {
        windowOuterWidth = window.outerWidth;
        var width = windowOuterWidth * window.devicePixelRatio;
        viewport = document.querySelector("meta[name=viewport]");
        viewport.setAttribute('content', 'width=' + width + ', viewport-fit=cover');

        if(resize)
            window.dispatchEvent(new Event('resize'));
    }
}

UpdateViewPortSize();

window.addEventListener("orientationchange", function() {
    UpdateViewPortSize();
}, false);

// Setup NoSleep
var noSleep = new NoSleep();

function keepScreenAwake()
{
    closeNav();
    noSleep.enable();
}

function checkForDefaultSettings()
{
    // MQTT
    if(localStorage.getItem("mqttHost") === null)
        localStorage.setItem("mqttHost", "broker.emqx.io");
    if(localStorage.getItem("mqttPort") === null)
        localStorage.setItem("mqttPort", "8084");
    if(localStorage.getItem("mqttTopic") === null)
        localStorage.setItem("mqttTopic", "revspace/sensors/dnrbtelem");
    if(localStorage.getItem("mqttUseTLS") === null)
        localStorage.setItem("mqttUseTLS", "true");
    
    // UI
    if(localStorage.getItem("ui_speed") === null)
        localStorage.setItem("ui_speed", "kmh" );

    if(localStorage.getItem("ui_distance") === null)
        localStorage.setItem("ui_distance", "km" );

    if(localStorage.getItem("ui_altitude") === null)
        localStorage.setItem("ui_altitude", "m" );

    if(localStorage.getItem("ui_current") === null)
        localStorage.setItem("ui_current", "a" );

    if(localStorage.getItem("ui_capacity") === null)
        localStorage.setItem("ui_capacity", "mah" );

    if(localStorage.getItem("ui_efficiency") === null)
        localStorage.setItem("ui_efficiency", "mahkm" );

    if(localStorage.getItem("ui_elevation_provider") === null)
        localStorage.setItem("ui_elevation_provider", "OpenTopoData" );
}

checkForDefaultSettings();

// Setup Sidebar
function openNav() {
    // If App is running standalone, then don't show the option to install
    document.getElementById("installhomelink").style.display = isRunningStandalone() ? "none" : "";

    // Not implemented yet
    //document.getElementById("uisettingslink").style.display = "none";
    document.getElementById("missionplannerlink").style.display = "none";
    document.getElementById("senduavcommandlink").style.display = "none";
    document.getElementById("inavsettingslink").style.display = "none";
    document.getElementById("installhomelink").style.display = "none";

    // Open menu
    document.getElementById("sideMenu").style.width = "100%";
}

function closeNav() {
    document.getElementById("sideMenu").style.width = "0";
}

function openBrokerSettings() {
    document.getElementById("brokerHost").value = localStorage.getItem("mqttHost");
    document.getElementById("brokerPort").value = localStorage.getItem("mqttPort");
    document.getElementById("brokerUser").value = localStorage.getItem("mqttUser");
    document.getElementById("brokerPass").value = localStorage.getItem("mqttPass");
    document.getElementById("brokerTopic").value = localStorage.getItem("mqttTopic");
    document.getElementById("brokerUseTLS").checked = (localStorage.getItem("mqttUseTLS") == "true");

    document.getElementById("brokerSettings").style.width = "100%";
}

function saveBrokerSettings()
{
    localStorage.setItem("mqttHost", document.getElementById("brokerHost").value);
    localStorage.setItem("mqttPort", document.getElementById("brokerPort").value);
    
    if(document.getElementById("brokerUser").value.length > 0)
        localStorage.setItem("mqttUser", document.getElementById("brokerUser").value);
    else
        localStorage.removeItem("mqttUser");

    if(document.getElementById("brokerUser").value.length > 0)
        localStorage.setItem("mqttPass", document.getElementById("brokerPass").value);
    else
        localStorage.removeItem("mqttPass");

    localStorage.setItem("mqttTopic", document.getElementById("brokerTopic").value);
    localStorage.setItem("mqttUseTLS", document.getElementById("brokerUseTLS").checked ? "true" : "false");

    if(mqttConnected )
        mqtt.disconnect();

    closeBrokerSettings();
    closeNav();
}

function resetBrokerSettings()
{
    MQTTSetDefaultSettings();

    if(mqttConnected )
        mqtt.disconnect();

    closeBrokerSettings();
    closeNav();
}

function closeBrokerSettings() {
    document.getElementById("brokerSettings").style.width = "0";
}





function openUISettings() 
{
    document.getElementById("ui_speed").value = localStorage.getItem("ui_speed");
    document.getElementById("ui_distance").value = localStorage.getItem("ui_distance");
    document.getElementById("ui_altitude").value = localStorage.getItem("ui_altitude");
    document.getElementById("ui_current").value = localStorage.getItem("ui_current");
    document.getElementById("ui_capacity").value = localStorage.getItem("ui_capacity");
    document.getElementById("ui_efficiency").value = localStorage.getItem("ui_efficiency");
    document.getElementById("ui_elevation_provider").value = localStorage.getItem("ui_elevation_provider");

    document.getElementById("uiSettings").style.width = "100%";
}

function saveUISettings()
{
    localStorage.setItem("ui_speed", document.getElementById("ui_speed").value );
    localStorage.setItem("ui_distance", document.getElementById("ui_distance").value );
    localStorage.setItem("ui_altitude", document.getElementById("ui_altitude").value );
    localStorage.setItem("ui_current", document.getElementById("ui_current").value );
    localStorage.setItem("ui_capacity", document.getElementById("ui_capacity").value );
    localStorage.setItem("ui_efficiency", document.getElementById("ui_efficiency").value );
    localStorage.setItem("ui_elevation_provider", document.getElementById("ui_elevation_provider").value );

    setUIUnits();
    closeUISettings();
    closeNav();
}

var uiElementsUnits = {
    distanceUnit: "km",
    distanceMetersToYardsFactor: 1.09361,
    distanceMetersToMilesFactor: 0.000621371,
    distanceMetersToFeetFactor: 3.28084,
    distanceMetersToNauticalMilesFactor: 0.000539957,
    speedUnit: "kmh",
    speedCmsToKmhFactor: 0.036,
    speedCmsToMphFactor: 0.0223694,
    speedCmsToKtFactor: 0.0194384,
    speedCmsToMsFactor: 0.01,
    altitudeUnit: "km",
    altitudeCmToFeetFactor: 30.48,
    altitudeCmToMFactor: 100,
    currentUnit: "a",
    capacityUnit: "mah",
};

function setUIUnits()
{
    // Set the UI elements
    uiElementsUnits.distanceUnit = localStorage.getItem("ui_distance");
    uiElementsUnits.speedUnit = localStorage.getItem("ui_speed");
    uiElementsUnits.altitudeUnit = localStorage.getItem("ui_altitude");
    uiElementsUnits.currentUnit = localStorage.getItem("ui_current");
    uiElementsUnits.capacityUnit = localStorage.getItem("ui_capacity");
    uiElementsUnits.efficiencyUnit = localStorage.getItem("ui_efficiency");

    if(uiElementsUnits.speedUnit == "kmh")
    {
        efis.SpeedUnitLabel = "Km/h";
        efis.SpeedUnitFactor = uiElementsUnits.speedCmsToKmhFactor;
    }
    else if(uiElementsUnits.speedUnit == "mph")
    {
        efis.SpeedUnitLabel = "Mph";
        efis.SpeedUnitFactor = uiElementsUnits.speedCmsToMphFactor;
    }
    else if(uiElementsUnits.speedUnit == "kt")
    {
        efis.SpeedUnitLabel = "Kt";
        efis.SpeedUnitFactor = uiElementsUnits.speedCmsToKtFactor;
    }
    else if(uiElementsUnits.speedUnit == "ms")
    {
        efis.SpeedUnitLabel = "M/s";
        efis.SpeedUnitFactor = uiElementsUnits.speedCmsToMsFactor;
    }

    if(uiElementsUnits.altitudeUnit == "m")
    {
        efis.AltitudeUnitLabel = "m";
        efis.AltitudeUnitFactor = uiElementsUnits.altitudeCmToMFactor;
        efis.AltitudeSmallTextNumber = 2;
    }
    else if(uiElementsUnits.altitudeUnit == "ft")
    {
        efis.AltitudeUnitLabel = "ft";
        efis.AltitudeUnitFactor = uiElementsUnits.altitudeCmToFeetFactor;
        efis.AltitudeSmallTextNumber = 3;
    }


}

setUIUnits();

function getRadioValue(radioName) {
    var ret = "";

    var radios = document.getElementsByName(radioName);

    for (i=0; i < radios.length; i++)
    {
        if(radios[i].checked)
        {
            ret = radios[i].value;
            break;
        }
    }
    return ret;
}

function setRadioValue(radioName, value) {
    var radios = document.getElementsByName(radioName);

    for (i=0; i < radios.length; i++)
    {
        if(radios[i].value == value)
        {
            radios[i].checked = true;
            break;
        }
    }
}

function closeUISettings() {
    document.getElementById("uiSettings").style.width = "0";
}


function reloadApplication()
{
    window.location.reload();
}

function isRunningStandalone()
{
    isInWebAppiOS = (window.navigator.standalone === true);
    isInWebAppChrome = (window.matchMedia('(display-mode: standalone)').matches);

    return isInWebAppiOS || isInWebAppChrome;
}

var newUIVersionAvailable = false;

function checkforNewUIVersion() {
    var xmlhttp = new XMLHttpRequest();

    xmlhttp.onreadystatechange = function() {
        if (xmlhttp.readyState == XMLHttpRequest.DONE) 
        {
            if (xmlhttp.status == 200) 
            {
                var jsonResponse = JSON.parse(xmlhttp.responseText);
                
                if(jsonResponse.lastVersion != currentVersion)
                {
                    newUIVersionAvailable = true;
                    console.log("New UI version available!");
                    document.getElementById("refreshMenuBadge").style.display = "inline";
                    document.getElementById("gearIconBadge").style.display = "inline";
                }

            }
            else 
            {
                console.log("Error checking for newer UI version. Status: " + xmlhttp.status);
                console.log("Response text: " + xmlhttp.responseText);
            }
        }
    };
    var uiversionurl = "uiversion.json?t=" + new Date().getTime();
    xmlhttp.open("GET", uiversionurl, true);
    xmlhttp.send();
}

document.addEventListener("touchmove", function(e){
    e.preventDefault();
},{passive: false});

window.onresize = function(event) {
    renderEFIS(data);
};

window.onload = function(event) {
    MQTTconnect();
    renderEFIS(data);
    updateDataView(data);
    drawAircraftOnMap(data);
    
    getUserLocation();
    drawUserOnMap(data);

    var timerEFIS = setInterval(function(){ 
        //simulateFlight();
        renderEFIS(data);
    }, 100); // 33 = 30fps, 66 = 15fps, 100 = 10fps, 200 = 5fps, 500 = 2fps

    var timerMapAndData = setInterval(function(){
        // Fast blinker should update as fast as the DataView panel
        blinkFastSwitch = !blinkFastSwitch;

        // Render stuff
        drawAircraftOnMap(data);
        drawAircraftPathOnMap(data);
        updateDataView(data);
    }, 250); // 33 = 30fps, 66 = 15fps, 100 = 10fps, 250 = 4fps, 500 = 2fps

    var timerOneSecond = setInterval(function(){ 
        // Update Flight Time and Power Time
        data.powerTime++;
        if(data.uavIsArmed)
        {
            data.flightTime++;

            // Remove the oldest waypoint if there are too many waypoints
            if(data.currentFlightWaypoints.length > 3600) // 3600 Waypoints means 1 hour of flight with 1 waypoint per second
                data.currentFlightWaypoints.shift();
    
            var wpCount = data.currentFlightWaypoints.length;
    
            var waypoint = {
                wpLatitude: data.gpsLatitude,
                wpLongitude: data.gpsLongitude,
            };
    
            data.currentFlightWaypoints[wpCount] = waypoint;
    
        }
            
    }, 1000);

    var lastTimeUIwasOpen = new Date().getTime();
    checkforNewUIVersion();

    var timerLowPriorityTasks = setInterval(function(){ 
        drawMissionOnMap(data);
        drawHomeOnMap(data);
        drawUserOnMap(data);

        // Center map
        if(user_moved_map == true)
            user_moved_map = false; // Set it false so the next time this routine will move the map
        else
            centerMap(data);

        // Update Mission WP Elevation data
        if(data.waypointCount > 0 && data.isCurrentMissionElevationSet == false
        && data.isWaypointMissionValid == 1 && updatingWpAltitudes == false
        && data.waypointCount == (data.currentMissionWaypoints.length - 1))
        {
            console.log("Trying to get WP Elevation data...");
            getMissionWaypointsAltitude();
        }

        // Check for ui update
        var uiUpdateNow = new Date().getTime();
        if(uiUpdateNow - lastTimeUIwasOpen > 15000) 
        {
            checkforNewUIVersion();
        }
        lastTimeUIwasOpen = uiUpdateNow;

            
    }, 5000);
    
    var timerBlinkSlow = setInterval(function(){ 
        blinkSlowSwitch = !blinkSlowSwitch;
    }, 2000);


}