// Setup MQTT

function MQTTSetDefaultSettings() 
{
    localStorage.setItem("mqttHost", "broker.emqx.io");
    localStorage.setItem("mqttPort", "8084");
    localStorage.setItem("mqttTopic", "revspace/sensors/dnrbtelem");
    localStorage.setItem("mqttUseTLS", "true");
    localStorage.removeItem("mqttUser");
    localStorage.removeItem("mqttPass");

}

// If there's no saved settings, save a default one
if (localStorage.getItem("mqttHost") === null)
    MQTTSetDefaultSettings() 

host = localStorage.getItem("mqttHost");	// hostname or IP address
port = parseInt(localStorage.getItem("mqttPort"));
useTLS = (localStorage.getItem("mqttUseTLS") == "true");
topic = localStorage.getItem("mqttTopic");		// topic to subscribe to
username = localStorage.getItem("mqttUser");
password = localStorage.getItem("mqttPass");
cleansession = true;

var mqttlog = new Array();

function mqttlogevent(eventDescription)
{
    var nowDate = new Date();
    mqttlog.push(nowDate.getTime().toString() + '|' + eventDescription);
}

function savemqttlog()
{
    var nowDate = new Date();
    var fileContent = mqttlog.join("\n");
    var bb = new Blob([ fileContent ], { type: 'text/plain' });
    var a = document.createElement('a');
    a.download = new Date().toISOString().replace(/:/g,"").replace(/-/g,"").substr(0,15) + '.txt';
    a.href = window.URL.createObjectURL(bb);
    a.click();
    closeNav();
}

var mqtt;
var reconnectTimeout = 2000;

var mqttConnected = false;
var lastMessageDate = new Date(2000, 1, 1);

function MQTTconnect() {
    host = localStorage.getItem("mqttHost");	// hostname or IP address
    port = parseInt(localStorage.getItem("mqttPort"));
    useTLS = (localStorage.getItem("mqttUseTLS") == "true");
    topic = localStorage.getItem("mqttTopic");		// topic to subscribe to
    username = localStorage.getItem("mqttUser");
    password = localStorage.getItem("mqttPass");
    cleansession = true;

    if (typeof path == "undefined") {
        path = '/mqtt';
    }

    mqtt = new Paho.MQTT.Client(
            host,
            port,
            path,
            "web_" + parseInt(Math.random() * 100, 10)
    );

    var options = {
        timeout: 3,
        useSSL: useTLS,
        cleanSession: cleansession,
        onSuccess: onConnect,
        onFailure: function (message) {
            var errmsg = "Connection failed: " + message.errorMessage + ". Retrying...";
            console.log(errmsg);
            mqttlogevent(errmsg);
            setTimeout(MQTTconnect, reconnectTimeout);
        }
    };

    mqtt.onConnectionLost = onConnectionLost;
    mqtt.onMessageArrived = onMessageArrived;

    if (username != null) {
        options.userName = username;
        options.password = password;
    }
    mqtt.connect(options);
}

function onConnect() {
    var errmsg = 'Connected to ' + host + ':' + port + path; 
    console.log(errmsg);
    // Connection succeeded; subscribe to our topic
    mqtt.subscribe(topic, {qos: 0});
    console.log(topic);
    mqttlogevent(errmsg + ' - ' + topic);
    mqttConnected = true;
}

function onConnectionLost(response) {
    setTimeout(MQTTconnect, reconnectTimeout);
    if(typeof responseObject !== 'undefined')
    {
        var errmsg = "Connection lost: " + responseObject.errorMessage + ". Reconnecting..."
        console.log(errmsg);
        mqttlogevent(errmsg);
    }
    mqttConnected = false;
};

function onMessageArrived(message) {

    var topic = message.destinationName;
    var payload = message.payloadString;

    console.log(payload);
    mqttlogevent(payload);
    lastMessageDate = new Date();
    parseTelemetryData(payload);
};

// MQTTconnect();

// Setup
var data = {
    rollAngle: 0, // decimal deg from -180.0 to 180.0
    pitchAngle: 0, // decimal deg from -90.0 to 90.0
    heading: 0, // decimal deg from -180.0 to 180.0
    altitude: 0, // int centimeters
    altitudeSeaLevel: 0, // int meters
    groundSpeed: 0, // int centimeters per second
    airSpeed: 0, // int centimeters per second
    verticalSpeed: 0, // int centimeters per second
    homeDirection: 0, // decimal deg from -180.0 to 180.0
    homeDistance: 0, // meters
    fuelPercent: 100,
    battCellVoltage: 4,
    batteryVoltage: 16,
    batteryCellCount: 4,
    currentDraw: 0, // Amps
    capacityDraw: 0, // mAh
    rssiPercent: 100,
    gpsSatCount: 0,
    gpsHDOP: 0,
    gpsLatitude: -23.5467,
    gpsLongitude: -46.6652,
    currentWaypointNumber: 0,
    waypointCount: 0,
    azimuth: 100,
    elevation: 6,
    dataTimestamp: new Date(2000, 1, 1),
    flightMode: "N/A",
    cellSignalStrength: 3,
    gps3DFix: 0,
    isHardwareHealthy: 0,
    uavIsArmed: 0,
    isWaypointMissionValid: 0,
    callsign: "UNKNOWN",
    powerTime: 0,
    flightTime: 0,
    isFailsafeActive: 0,
    currentMissionWaypoints: new Array(),
    homeLatitude: 0,
    homeLongitude: 0,
    homeAltitudeSL: 0,
    userLatitude: 0,
    userLongitude: 0,
    userHeading: 0,
    userAltitudeSL: 0,
    currentFlightWaypoints: new Array(),
    throttlePercent: 0,
    isAutoThrottleActive: 0,
    navState: 0,
    mWhDraw: 0,
    isCurrentMissionElevationSet: false,
    gpsGroundCourse: 0,
    estimations: {
        gpsLatitude: 0,
        gpsLongitude: 0,
        homeDistance: 0,
        capacityDraw: 0,
        rollAngle: 0,
        pitchAngle: 0,
        heading: 0,
        altitude: 0,
        groundSpeed: 0,
        verticalSpeed: 0,
    },
    lastMessage: {
        rollAngle: 0,
        pitchAngle: 0,
        heading: 0,
        altitude: 0,
        groundSpeed: 0,
        verticalSpeed: 0,
    }
};

function parseWaypointMessage(payload) {
    // Don't update mission while fetching elevation data from server.
    if(updatingWpAltitudes)
        return;

    var wpno = 0;
    var arrPayload = payload.split(",");
    
    var waypoint = {
        waypointNumber: 0,
        wpAction: 0,
        wpLatitude: 0,
        wpLongitude: 0,
        wpAltitude: 0,
        p1: 0,
        p2: 0,
        p3: 0,
        flag: 0,
        elevation: 0,
    };
    
    for(var i=0; i < arrPayload.length; i++) {
        if(arrPayload[i] == "")
            continue;

        var arrData = arrPayload[i].split(":");
        
        switch(arrData[0]) {
            case "wpno":
                waypoint.waypointNumber = parseInt(arrData[1]);
                wpno = parseInt(arrData[1]);
                break;
            case "la":
                waypoint.wpLatitude = parseInt(arrData[1]) / 10000000.0;
                break;
            case "lo":
                waypoint.wpLongitude = parseInt(arrData[1]) / 10000000.0;
                break;
            case "ac":
                waypoint.wpAction = parseInt(arrData[1]);
                break;
            case "al":
                waypoint.wpAltitude = parseInt(arrData[1]);
                break;
            case "p1":
                waypoint.p1 = parseInt(arrData[1]);
                break;
            case "p2":
                waypoint.p2 = parseInt(arrData[1]);
                break;
            case "p3":
                waypoint.p3 = parseInt(arrData[1]);
                break;
            case "f":
                waypoint.flag = parseInt(arrData[1]);
                break;
            case "el":
                waypoint.elevation = parseInt(arrData[1]);
                break;
            default:
                break;
        }
    }

    // Check if the loaded waypoint has the same coordinates as the previous one, and change only if it's different
    if(data.currentMissionWaypoints.length <= wpno)
    {
        data.currentMissionWaypoints[wpno] = waypoint;
        if(wpno > 0)
            data.isCurrentMissionElevationSet = false;
        return;
    }
    else if(data.currentMissionWaypoints[wpno].wpLatitude != waypoint.wpLatitude 
    || data.currentMissionWaypoints[wpno].wpLongitude != waypoint.wpLongitude 
    || data.currentMissionWaypoints[wpno].wpAltitude != waypoint.wpAltitude 
    )
    {
        data.currentMissionWaypoints[wpno] = waypoint;
        if(wpno > 0)
            data.isCurrentMissionElevationSet = false;
        return;
    }
}

function clearCurrentMissionWaypoints() {
    data.currentMissionWaypoints = new Array();
    data.isCurrentMissionElevationSet = false;
}

function flightModeIdToName(flightModeId)
{
    switch(flightModeId)
    {
        case 1:
            return "MANUAL";
            break;
        case 2:
            return "RTH";
            break;
        case 3:
            return "A+PH";
            break;
        case 4:
            return "POS H";
            break;
        case 5:
            return "3CRS";
            break;
        case 6:
            return "CRS";
            break;
        case 7:
            return "WP";
            break;
        case 8:
            return "ALT H";
            break;
        case 9:
            return "ANGLE";
            break;
        case 10:
            return "HORIZON";
            break;
        case 11:
            return "ACRO";
            break;
        default:
            return "";
    }
}

function parseStandardTelemetryMessage(payload)
{
    var arrPayload = payload.split(",");
    for(var i=0; i < arrPayload.length; i++) {
        if(arrPayload[i] == "")
            continue;

        var arrData = arrPayload[i].split(":");
        switch(arrData[0]) {
            case "ran":
                data.lastMessage.rollAngle = data.rollAngle;
                data.rollAngle = parseFloat(arrData[1]) / 10.0;
                break;
            case "pan":
                data.lastMessage.pitchAngle = data.pitchAngle;
                data.pitchAngle = parseFloat(arrData[1]) / 10.0;
                break;
            case "hea":
                data.lastMessage.heading = data.heading;
                data.heading = parseFloat(arrData[1]);
                break;
            case "alt":
                data.lastMessage.altitude = data.altitude;
                data.altitude = parseInt(arrData[1]);
                break;
            case "asl":
                data.altitudeSeaLevel = parseInt(arrData[1]);
                break;
            case "gsp":
                data.lastMessage.groundSpeed = data.groundSpeed;
                data.groundSpeed = parseInt(arrData[1]);
                break;
            case "vsp":
                data.lastMessage.verticalSpeed = data.verticalSpeed;
                data.verticalSpeed = parseInt(arrData[1]);
                break;
            case "hdr":
                data.homeDirection = parseInt(arrData[1]);
                break;
            case "hds":
                data.homeDistance = parseInt(arrData[1]);
                break;
            case "cud":
                data.currentDraw = parseInt(arrData[1]) / 100.0;
                break;
            case "cad":
                data.capacityDraw = parseInt(arrData[1]);
                break;
            case "rsi":
                data.rssiPercent = parseInt(arrData[1]);
                break;
            case "gsc":
                data.gpsSatCount = parseInt(arrData[1]);
                break;
            case "gla":
                data.gpsLatitude = parseInt(arrData[1]) / 10000000.0;
                break;
            case "glo":
                data.gpsLongitude = parseInt(arrData[1]) / 10000000.0;
                break;
            case "ghp":
                data.gpsHDOP = parseInt(arrData[1]) / 100.0;
                break;
            case "acv":
                data.battCellVoltage = parseInt(arrData[1]) / 100.0;
                break;
            case "bpv":
                data.batteryVoltage = parseInt(arrData[1]) / 100.0;
                break;
            case "bcc":
                data.batteryCellCount = parseInt(arrData[1]);
                break;
            case "bfp":
                data.fuelPercent = parseInt(arrData[1]);
                break;
            case "css":
                data.cellSignalStrength = parseInt(arrData[1]);
                break;
            case "ftm":
                data.flightMode = flightModeIdToName(parseInt(arrData[1]));
                break;
            case "cwn":
                data.currentWaypointNumber = parseInt(arrData[1]);
                break;
            case "wpc":
                if(parseInt(arrData[1]) != data.waypointCount)
                    clearCurrentMissionWaypoints();

                data.waypointCount = parseInt(arrData[1]);
                break;
            case "3df":
                data.gps3DFix = parseInt(arrData[1]);
                break;
            case "hwh":
                data.isHardwareHealthy = parseInt(arrData[1]);
                break;
            case "arm":
                data.uavIsArmed = parseInt(arrData[1]);
                break;
            case "wpv":
                data.isWaypointMissionValid = parseInt(arrData[1]);
                break;
            case "cs":
                data.callsign = arrData[1];
                break;
            case "ont":
                data.powerTime = parseInt(arrData[1]);
                break;
            case "flt":
                data.flightTime = parseInt(arrData[1]);
                break;
            case "fs":
                data.isFailsafeActive = parseInt(arrData[1]);
                break;
            case "hla":
                data.homeLatitude = parseInt(arrData[1]) / 10000000.0;
                break;
            case "hlo":
                data.homeLongitude = parseInt(arrData[1]) / 10000000.0;
                break;
            case "hal":
                data.homeAltitudeSL = parseInt(arrData[1]) / 100.0; // Home comes in centimeters
                break;
            case "ont":
                data.powerTime = parseInt(arrData[1]);
                break;
            case "flt":
                data.flightTime = parseInt(arrData[1]);
                break;
            case "trp":
                data.throttlePercent = parseInt(arrData[1]);
                break;
            case "att":
                data.isAutoThrottleActive = parseInt(arrData[1]);
                break;
            case "nvs":
                data.navState = parseInt(arrData[1]);
                break;
            case "whd":
                data.mWhDraw = parseInt(arrData[1]);
                break;
            case "ggc":
                data.gpsGroundCourse = parseInt(arrData[1]);
                break;
            default:
                break;
        }
    }
}

function parseTelemetryData(payload) {
    var arrPayload = payload.split(",");

    if(arrPayload.length > 0) {
        // Check if it's a Waypoint frame or a regular telemetry frame
        if(payload.startsWith("wpno:"))
            parseWaypointMessage(payload);
        else
            parseStandardTelemetryMessage(payload);
    }


    data.dataTimestamp = new Date();
}

function estimatePosition()
{
    var dateNow = new Date();
    var timeSinceLastFrame = dateNow - lastMessageDate;

    if(timeSinceLastFrame > 2 * 60 * 1000) // Stop estimating if more than 2 minutes without messages
    {
        data.estimations.gpsLatitude = data.gpsLatitude;
        data.estimations.gpsLongitude = data.gpsLongitude;
        data.estimations.homeDistance = data.homeDistance;
        data.estimations.capacityDraw = data.capacityDraw;
        return;
    }

    // Estimate aircraft position based on speed and course
    if(data.groundSpeed > 0)
    {
        // Calculate how much distance it travels on that time and speed
        var distance = (data.groundSpeed / 100); // meters in one second
        distance = distance * (timeSinceLastFrame / 1000);

        // Calculate new estimated coordinates
        var estimatedCoordinates = DestinationCoordinates(data.gpsLatitude, data.gpsLongitude, data.gpsGroundCourse, distance);
        data.estimations.gpsLatitude = estimatedCoordinates.lat;
        data.estimations.gpsLongitude = estimatedCoordinates.lng;

        // Calculate home distance
        data.estimations.homeDistance = getDistanceBetweenTwoPoints(data.homeLatitude, data.homeLongitude, estimatedCoordinates.lat, estimatedCoordinates.lng);
    }

    // Estimate capacity consumption based on the power consumption
    var usedCapacity = (data.currentDraw / 3600) * 1000; // milliamps-hour in one second
    usedCapacity = usedCapacity * (timeSinceLastFrame / 1000);
    data.estimations.capacityDraw = data.capacityDraw + usedCapacity;
}

function rangeNumbers(in_number, in_min, in_max, out_min, out_max) {
    return (in_number - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

function rangeNumbers360(in_number, in_min, in_max, out_min, out_max) {
    var retval = 0;

    if(out_min >= 330 && out_max <= 30) // Moving from left to right passing thru 0
    {
        out_max = out_max + 360;
        retval = rangeNumbers(in_number, in_min, in_max, out_min, out_max);
        if(retval >= 360)
            retval = retval - 360;
    }
    else if(out_min <= 30 && out_max >= 330) // Moving from right to left passing thru 0
    {
        out_min = out_min + 360;
        retval = rangeNumbers(in_number, in_min, in_max, out_min, out_max);
        if(retval >= 360)
            retval = retval - 360;
    }
    else
        retval = rangeNumbers(in_number, in_min, in_max, out_min, out_max);

    return retval;
}

function estimateEfis()
{
    var dateNow = new Date();
    var timeSinceLastFrame = dateNow - lastMessageDate;

    if(timeSinceLastFrame > pageSettings.messageInterval) // Stop estimating
    {
        
        data.lastMessage.altitude = data.altitude;
        data.lastMessage.rollAngle = data.rollAngle;
        data.lastMessage.pitchAngle = data.pitchAngle;
        data.lastMessage.groundSpeed = data.groundSpeed;
        data.lastMessage.verticalSpeed = data.verticalSpeed;
        data.lastMessage.heading = data.heading;

        data.estimations.altitude = data.altitude;
        data.estimations.rollAngle = data.rollAngle;
        data.estimations.pitchAngle = data.pitchAngle;
        data.estimations.groundSpeed = data.groundSpeed;
        data.estimations.verticalSpeed = data.verticalSpeed;
        data.estimations.heading = data.heading;
        return;
    }

    // This is the percentage of the movement to the last frame
    var movePercent = (timeSinceLastFrame / pageSettings.messageInterval) * 100;
    
    data.estimations.altitude = rangeNumbers(movePercent, 0, 100, data.lastMessage.altitude, data.altitude);
    data.estimations.rollAngle = rangeNumbers(movePercent, 0, 100, data.lastMessage.rollAngle, data.rollAngle);
    data.estimations.pitchAngle = rangeNumbers(movePercent, 0, 100, data.lastMessage.pitchAngle, data.pitchAngle);
    data.estimations.groundSpeed = rangeNumbers(movePercent, 0, 100, data.lastMessage.groundSpeed, data.groundSpeed);
    data.estimations.verticalSpeed = rangeNumbers(movePercent, 0, 100, data.lastMessage.verticalSpeed, data.verticalSpeed);
    data.estimations.heading = rangeNumbers360(movePercent, 0, 100, data.lastMessage.heading, data.heading);


}
