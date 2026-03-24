// Setup MQTT

// Declare connection variables (re-read from localStorage in MQTTconnect)
let host, port, useTLS, topic, username, password, cleansession, path;

export function MQTTSetDefaultSettings()
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

let mqttlog = new Array();
export let isPlayingLogFile = false;

// Called by PageScripts for each live MQTT message (line = "timestamp|payload").
// Used to record messages into the active session.
let onMessageCallback = null;
export function setOnMessageCallback(fn) { onMessageCallback = fn; }

// Called by PageScripts when a session replay ends.
let onReplayStopCallback = null;
export function setOnReplayStop(fn) { onReplayStopCallback = fn; }

function mqttlogevent(eventDescription)
{
    var nowDate = new Date();
    mqttlog.push(nowDate.getTime().toString() + '|' + eventDescription);
}

export function savemqttlog()
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

let replayIndex = 0;
let replayInitialTimestamp = 0;
let replayCurrentTimestamp = 0;
let firstReplayTimestamp = 0;
let lastReplayTimestamp = 0;
let timeSinceBeggining = 0;
let totalReplayTime = 0;
export let playbackPercent = 0;

export function replaymqttlog()
{
    var inputFileElement = document.createElement('input');
    inputFileElement.type="file";
    inputFileElement.addEventListener("change", function () {
        if (this.files && this.files[0]) {
          var myFile = this.files[0];
          var reader = new FileReader();
          
          reader.addEventListener('load', function (e) {
            mqttConnected = true;
            isPlayingLogFile = true;
            mqtt.disconnect();

            mqttlog = e.target.result.split('\n');
            replayIndex = 0;

            // Update UI
            document.getElementById("playbackcontainer").style.display = "inherit";
            document.getElementById("btStopReplay").style.display = "block";
            resetDataObject();
          });
          
          reader.readAsBinaryString(myFile);
        }   
      });
    inputFileElement.click();
 
}

export function stopreplaymqttlog()
{
    mqttlog = new Array();
    resetDataObject();
    isPlayingLogFile = false;
    replayIndex = 0;
    mqttConnected = false;
    document.getElementById("playbackcontainer").style.display = "none";
    document.getElementById("btStopReplay").style.display = "none";
    MQTTconnect();
    if (onReplayStopCallback) onReplayStopCallback();
}

// Start a replay from an array of "timestamp|payload" lines (loaded from IndexedDB).
export function replayFromSessionMessages(lines) {
    if (!lines || lines.length === 0) {
        alert("This session has no recorded messages.");
        return;
    }
    mqttConnected = true;
    isPlayingLogFile = true;
    if (mqtt) mqtt.disconnect();
    mqttlog = lines.slice();
    replayIndex = 0;
    document.getElementById("playbackcontainer").style.display = "inherit";
    document.getElementById("btStopReplay").style.display = "block";
    resetDataObject();
}

// Fast-forward all session lines through the parser to restore the last known state.
export function restoreFromSessionMessages(lines) {
    resetDataObject();
    if (!lines || lines.length === 0) return;
    for (var i = 0; i < lines.length; i++) {
        var parts = lines[i].split('|');
        if (parts.length < 2) continue;
        var payload = parts[1];
        if (!payload || payload.startsWith('Connect')) continue;
        parseTelemetryData(payload);
    }
}

function setplaybackpercent(percent)
{
    var percentFrame = (totalReplayTime / 100) * percent;
    // Round it to a value divisible by 1000
    percentFrame = Math.floor(percentFrame / 1000) * 1000;
    timeSinceBeggining = percentFrame;
}

function updatePlaybackTimers(percent)
{
    var percentFrame = (totalReplayTime / 100) * percent;
    // Round it to a value divisible by 1000
    percentFrame = Math.floor(percentFrame / 1000) * 1000;
    //timeSinceBeggining = percentFrame;

    document.getElementById("currentPlaybackTime").innerHTML = secondsToNiceTime(percentFrame / 1000);
}

let isDraggingSliderReplay = false;

document.getElementById("sldrReplay").onmouseup = function() {
    setplaybackpercent(this.value);
    isDraggingSliderReplay = false;
}

document.getElementById("sldrReplay").ontouchend = function() {
    setplaybackpercent(this.value);
    isDraggingSliderReplay = false;
}

document.getElementById("sldrReplay").oninput = function() {
    isDraggingSliderReplay = true;
    updatePlaybackTimers(this.value);
}

var timerReplay = setInterval(function() {
    if(isPlayingLogFile)
    {
        var replayFrame = "";
        var nowDate = new Date();
        replayCurrentTimestamp = nowDate.getTime().toString();
        
        if(replayIndex == 0)
        {
            // Get the first and last timestamp so we can calculate time
            replayFrame = mqttlog[replayIndex];
            firstReplayTimestamp = parseInt(replayFrame.split('|')[0]);
            replayFrame = mqttlog[mqttlog.length - 1];
            lastReplayTimestamp = parseInt(replayFrame.split('|')[0]);
            replayInitialTimestamp = replayCurrentTimestamp;
            timeSinceBeggining = replayCurrentTimestamp - replayInitialTimestamp;
            totalReplayTime = lastReplayTimestamp - firstReplayTimestamp;
            playbackPercent = 0;
            document.getElementById("maxPlaybackTime").innerHTML = secondsToNiceTime(totalReplayTime / 1000);
        }

        timeSinceBeggining += 1000;
        
        // Update UI
        if(!isDraggingSliderReplay)
            document.getElementById("currentPlaybackTime").innerHTML = secondsToNiceTime(timeSinceBeggining / 1000);
        
        if(document.getElementById("sldrReplay").value != playbackPercent.toFixed(1) && !isDraggingSliderReplay)
            document.getElementById("sldrReplay").value = playbackPercent;

        while(true)
        {
            var arrReplayFrame = mqttlog[replayIndex].split('|');
            var replayFrameTimestamp = arrReplayFrame[0];
            var replayFrameData = arrReplayFrame[1];

            // Check if it has been passed enough time so we can use this frame
            var timeFrame = replayFrameTimestamp - firstReplayTimestamp;
            playbackPercent = (timeSinceBeggining / totalReplayTime) * 100;
            console.log("Frame time: " + timeFrame + " - Replay time: " + timeSinceBeggining + " - Playback %: " + playbackPercent.toFixed(2));

            if(timeFrame <= timeSinceBeggining)
            {
                // Process frame
                console.log("Replay: " + replayFrameData);

                if(!replayFrameData.startsWith('Connect'))
                {
                    lastMessageDate = new Date();
                    mqttConnected = true;
                    parseTelemetryData(replayFrameData);
                }

                // Next frame
                replayIndex++;

                if(replayIndex >= mqttlog.length)
                {
                    // Replay ended
                    stopreplaymqttlog();
                    break;
                }
            }
            else
            {
                break;
            }
        }
    }

}, 1000);

export let mqtt;
let reconnectTimeout = 2000;

export let mqttConnected = false;
export let lastMessageDate = new Date(2000, 1, 1);

export function MQTTconnect() {
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
            if(!isPlayingLogFile)
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
    if(!isPlayingLogFile)
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
    if(!isPlayingLogFile)
    {
        var topic = message.destinationName;
        var payload = message.payloadString;

        console.log(payload);
        var line = new Date().getTime().toString() + '|' + payload;
        mqttlog.push(line);
        lastMessageDate = new Date();
        parseTelemetryData(payload);
        if (onMessageCallback) onMessageCallback(line);
    }
};

// MQTTconnect();
export let data = {};
export function resetDataObject()
{
    data = {
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
}
// Setup
resetDataObject();

export function inRange(val, min, max) {
    return !isNaN(val) && val >= min && val <= max;
}

function parseWaypointMessage(payload) {
    // Don't update mission while fetching elevation data from server.
    if(updatingWpAltitudes)
        return;

    var wpno = null;
    var rawLa = null, rawLo = null;

    var waypoint = {
        waypointNumber: 0,
        wpAction: 1,
        wpLatitude: 0,
        wpLongitude: 0,
        wpAltitude: 0,
        p1: 0,
        p2: 0,
        p3: 0,
        flag: 0,
        elevation: 0,
    };

    var arrPayload = payload.split(",");
    for(var i=0; i < arrPayload.length; i++) {
        if(arrPayload[i] == "")
            continue;

        var arrData = arrPayload[i].split(":");
        var raw;

        switch(arrData[0]) {
            case "wpno":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 255)) wpno = raw;
                break;
            case "la":
                rawLa = parseInt(arrData[1]);
                break;
            case "lo":
                rawLo = parseInt(arrData[1]);
                break;
            case "ac":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 1, 8)) waypoint.wpAction = raw;
                break;
            case "al":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 60000)) waypoint.wpAltitude = raw;
                break;
            case "p1":
                raw = parseInt(arrData[1]);
                if(inRange(raw, -32768, 32767)) waypoint.p1 = raw;
                break;
            case "p2":
                raw = parseInt(arrData[1]);
                if(inRange(raw, -32768, 32767)) waypoint.p2 = raw;
                break;
            case "p3":
                raw = parseInt(arrData[1]);
                if(inRange(raw, -32768, 32767)) waypoint.p3 = raw;
                break;
            case "f":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 255)) waypoint.flag = raw;
                break;
            case "el":
                raw = parseInt(arrData[1]);
                if(!isNaN(raw)) waypoint.elevation = raw;
                break;
            default:
                break;
        }
    }

    // Reject the waypoint if the index is invalid
    if(wpno === null) return;
    waypoint.waypointNumber = wpno;

    // Both coordinates must be present and valid to place the waypoint
    var laValid = rawLa !== null && inRange(rawLa, -900000000, 900000000);
    var loValid = rawLo !== null && inRange(rawLo, -1800000000, 1800000000);
    if(!laValid || !loValid) return;
    waypoint.wpLatitude = rawLa / 10000000.0;
    waypoint.wpLongitude = rawLo / 10000000.0;

    // Store only if this is a new slot or the coordinates changed
    if(data.currentMissionWaypoints.length <= wpno)
    {
        data.currentMissionWaypoints[wpno] = waypoint;
        if(wpno > 0)
            data.isCurrentMissionElevationSet = false;
        return;
    }
    else if(data.currentMissionWaypoints[wpno] !== undefined
        && (data.currentMissionWaypoints[wpno].wpLatitude != waypoint.wpLatitude
        || data.currentMissionWaypoints[wpno].wpLongitude != waypoint.wpLongitude
        || data.currentMissionWaypoints[wpno].wpAltitude != waypoint.wpAltitude)
    )
    {
        data.currentMissionWaypoints[wpno] = waypoint;
        if(wpno > 0)
            data.isCurrentMissionElevationSet = false;
        return;
    }
}

export function clearCurrentMissionWaypoints() {
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

    // GPS and home coordinate pairs are collected first and validated together after the loop
    var rawGla = null, rawGlo = null;
    var rawHla = null, rawHlo = null;

    for(var i=0; i < arrPayload.length; i++) {
        if(arrPayload[i] == "")
            continue;

        var arrData = arrPayload[i].split(":");
        var raw;

        switch(arrData[0]) {
            case "ran":
                raw = parseFloat(arrData[1]);
                if(inRange(raw, -1800, 1800)) {
                    data.lastMessage.rollAngle = data.rollAngle;
                    data.rollAngle = raw / 10.0;
                }
                break;
            case "pan":
                raw = parseFloat(arrData[1]);
                if(inRange(raw, -900, 900)) {
                    data.lastMessage.pitchAngle = data.pitchAngle;
                    data.pitchAngle = raw / 10.0;
                }
                break;
            case "hea":
                raw = parseFloat(arrData[1]);
                if(inRange(raw, 0, 359)) {
                    data.lastMessage.heading = data.heading;
                    data.heading = raw;
                }
                break;
            case "alt":
                raw = parseInt(arrData[1]);
                if(inRange(raw, -1000000, 10000000)) {
                    data.lastMessage.altitude = data.altitude;
                    data.altitude = raw;
                }
                break;
            case "asl":
                raw = parseInt(arrData[1]);
                if(inRange(raw, -500, 9000))
                    data.altitudeSeaLevel = raw;
                break;
            case "gsp":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 15000)) {
                    data.lastMessage.groundSpeed = data.groundSpeed;
                    data.groundSpeed = raw;
                }
                break;
            case "vsp":
                raw = parseInt(arrData[1]);
                if(inRange(raw, -60000, 60000)) {
                    data.lastMessage.verticalSpeed = data.verticalSpeed;
                    data.verticalSpeed = raw;
                }
                break;
            case "hdr":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 359))
                    data.homeDirection = raw;
                break;
            case "hds":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 20000000))
                    data.homeDistance = raw;
                break;
            case "cud":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 50000))
                    data.currentDraw = raw / 100.0;
                break;
            case "cad":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 100000))
                    data.capacityDraw = raw;
                break;
            case "rsi":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 100))
                    data.rssiPercent = raw;
                break;
            case "gsc":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 50))
                    data.gpsSatCount = raw;
                break;
            case "gla":
                rawGla = parseInt(arrData[1]);
                break;
            case "glo":
                rawGlo = parseInt(arrData[1]);
                break;
            case "ghp":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 9999))
                    data.gpsHDOP = raw / 100.0;
                break;
            case "acv":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 500))
                    data.battCellVoltage = raw / 100.0;
                break;
            case "bpv":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 6000))
                    data.batteryVoltage = raw / 100.0;
                break;
            case "bcc":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 1, 12))
                    data.batteryCellCount = raw;
                break;
            case "bfp":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 100))
                    data.fuelPercent = raw;
                break;
            case "css":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 3))
                    data.cellSignalStrength = raw;
                break;
            case "ftm":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 1, 11))
                    data.flightMode = flightModeIdToName(raw);
                break;
            case "cwn":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 255))
                    data.currentWaypointNumber = raw;
                break;
            case "wpc":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 256)) {
                    if(raw != data.waypointCount)
                        clearCurrentMissionWaypoints();
                    data.waypointCount = raw;
                }
                break;
            case "3df":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1)
                    data.gps3DFix = raw;
                break;
            case "hwh":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1)
                    data.isHardwareHealthy = raw;
                break;
            case "arm":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1)
                    data.uavIsArmed = raw;
                break;
            case "wpv":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1)
                    data.isWaypointMissionValid = raw;
                break;
            case "cs":
                if(/^[A-Za-z0-9_-]{1,16}$/.test(arrData[1]))
                    data.callsign = arrData[1];
                break;
            case "ont":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 172800))
                    data.powerTime = raw;
                break;
            case "flt":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 86400))
                    data.flightTime = raw;
                break;
            case "hla":
                rawHla = parseInt(arrData[1]);
                break;
            case "hlo":
                rawHlo = parseInt(arrData[1]);
                break;
            case "hal":
                raw = parseInt(arrData[1]);
                if(inRange(raw, -50000, 900000))
                    data.homeAltitudeSL = raw / 100.0; // Home comes in centimeters
                break;
            case "trp":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 100))
                    data.throttlePercent = raw;
                break;
            case "att":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1)
                    data.isAutoThrottleActive = raw;
                break;
            case "nvs":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 30))
                    data.navState = raw;
                break;
            case "whd":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 1000000))
                    data.mWhDraw = raw;
                break;
            case "ggc":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 0, 359))
                    data.gpsGroundCourse = raw;
                break;
            case "fs":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1)
                    data.isFailsafeActive = raw;
                break;
            case "mfr":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 100, 10000))
                    pageSettings.messageInterval = raw;
                break;
            default:
                break;
        }
    }

    // Validate GPS coordinate pairs together after the loop.
    // When both arrive in the same message, both must be valid or both are discarded.
    // When only one arrives (the other didn't change), validate independently.
    if(rawGla !== null || rawGlo !== null) {
        var glaValid = rawGla !== null && inRange(rawGla, -900000000, 900000000);
        var gloValid = rawGlo !== null && inRange(rawGlo, -1800000000, 1800000000);
        if(rawGla !== null && rawGlo !== null) {
            if(glaValid && gloValid) {
                data.gpsLatitude  = rawGla / 10000000.0;
                data.gpsLongitude = rawGlo / 10000000.0;
            }
        } else {
            if(glaValid) data.gpsLatitude  = rawGla / 10000000.0;
            if(gloValid) data.gpsLongitude = rawGlo / 10000000.0;
        }
    }

    if(rawHla !== null || rawHlo !== null) {
        var hlaValid = rawHla !== null && inRange(rawHla, -900000000, 900000000);
        var hloValid = rawHlo !== null && inRange(rawHlo, -1800000000, 1800000000);
        if(rawHla !== null && rawHlo !== null) {
            if(hlaValid && hloValid) {
                data.homeLatitude  = rawHla / 10000000.0;
                data.homeLongitude = rawHlo / 10000000.0;
            }
        } else {
            if(hlaValid) data.homeLatitude  = rawHla / 10000000.0;
            if(hloValid) data.homeLongitude = rawHlo / 10000000.0;
        }
    }
}

export function parseTelemetryData(payload) {
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

export function estimatePosition()
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

export function rangeNumbers(in_number, in_min, in_max, out_min, out_max) {
    return (in_number - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

export function rangeNumbers360(in_number, in_min, in_max, out_min, out_max) {
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

export function estimateEfis()
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

// Timing and refresh intervals (consumed by estimateEfis and PageScripts timers)
export let pageSettings = {
    efisRefreshInterval: 100,
    mapAndDataRefreshInterval: 250,
    lowPriorityTasksInterval: 10000,
    messageInterval: 1000,
};

// Flag set by MapScripts while an elevation API request is in flight.
// Read by parseWaypointMessage and InfoPanelScripts status bar.
export let updatingWpAltitudes = false;
export function setUpdatingWpAltitudes(val) { updatingWpAltitudes = val; }

// Haversine distance between two GPS coordinates, returns metres.
// Used internally by estimatePosition and by InfoPanelScripts.
export function getDistanceBetweenTwoPoints(lat1, lon1, lat2, lon2)
{
    var R = 6371; // km
    var dLat = (lat2 - lat1) * Math.PI / 180;
    var dLon = (lon2 - lon1) * Math.PI / 180;
    var a1 = AngleToRadians(lat1);
    var a2 = AngleToRadians(lat2);

    var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
    Math.sin(dLon/2) * Math.sin(dLon/2) * Math.cos(a1) * Math.cos(a2);
    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    var d = R * c;
    return d * 1000; // metres
}

// Vincenty destination point given a start, bearing and distance.
// Used internally by estimatePosition and by MapScripts drawCourseLineOnMap.
export function DestinationCoordinates(lat1, lon1, brng, dist) {
    var a = 6378137,
        b = 6356752.3142,
        f = 1 / 298.257223563,
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
    }
    var tmp = sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1,
        lat2 = Math.atan2(sinU1 * cosSigma + cosU1 * sinSigma * cosAlpha1, (1 - f) * Math.sqrt(sinAlpha * sinAlpha + tmp * tmp)),
        lambda = Math.atan2(sinSigma * sinAlpha1, cosU1 * cosSigma - sinU1 * sinSigma * cosAlpha1),
        C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha)),
        L = lambda - (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));
    return {
        lat: RadiansToAngle(lat2),
        lng: lon1 + RadiansToAngle(L),
    };
}

export function secondsToNiceTime(seconds)
{
    if(seconds > 30 * 60 * 60 * 24) // More than 30 days means never
        return "Never";

    var minutes = 0;
    var hours = 0;
    var days = 0;

    if(seconds > (60 * 60 * 24))
    {
        days = Math.floor(seconds / (60 * 60 * 24));
        seconds = seconds - (days * 60 * 60 * 24);
    }

    if(seconds > (60 * 60))
    {
        hours = Math.floor(seconds / (60 * 60));
        seconds = seconds - (hours * 60 * 60);
    }

    if(seconds > 60)
    {
        minutes = Math.floor(seconds / 60);
        seconds = seconds - (minutes * 60);
    }

    var ret = "";
    if(days > 0)
        ret += days + "d ";

    if(hours > 0)
        ret += hours + "h ";

    if(minutes > 0)
        ret += minutes + "m ";

    ret += seconds.toFixed(0) + "s";
    return ret;
}

// AngleToRadians / RadiansToAngle are defined in EfisScripts.js which is loaded
// as a plain script before modules run, so they are available as globals here.
function AngleToRadians(angle) { return angle * Math.PI / 180; }
function RadiansToAngle(radians) { return radians * 180 / Math.PI; }
