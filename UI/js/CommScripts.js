// Setup MQTT

// Declare connection variables (re-read from localStorage in MQTTconnect)
let host, port, useTLS, topic, commandTopic, username, password, cleansession, path;

export function MQTTSetDefaultSettings()
{
    localStorage.setItem("mqttHost", "broker.emqx.io");
    localStorage.setItem("mqttPort", "8084");
    localStorage.setItem("mqttTopic", "bulletgcss/telem/your_callsign");
    localStorage.setItem("mqttCommandTopic", "bulletgcss/cmd/your_callsign");
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
    topic = localStorage.getItem("mqttTopic");		// uplink topic (telemetry) to subscribe to
    commandTopic = localStorage.getItem("mqttCommandTopic");	// downlink topic (commands) to publish to
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
    // Subscribe to uplink (telemetry) topic
    mqtt.subscribe(topic, {qos: 0});
    // Re-subscribe to any secondary monitored aircraft topics
    for (var t in otherAircraft) {
        mqtt.subscribe(t, { qos: 0 });
        console.log('Secondary topic: ' + t);
    }
    console.log('Uplink topic: ' + topic);
    console.log('Downlink topic: ' + commandTopic);
    mqttlogevent(errmsg + ' - ' + topic);
    mqttConnected = true;
}

// ── Command tracking ─────────────────────────────────────────────────────────

function generateCid() {
    var chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    var result = '';
    for (var i = 0; i < 6; i++)
        result += chars.charAt(Math.floor(Math.random() * chars.length));
    return result;
}

// pendingCommands: { [cid]: { type, messageCount } }
var pendingCommands = {};

// commandHistory: array of { cid, type, timestamp, status } — newest first
export var commandHistory = [];

function getNextCommandSeq() {
    var seq = parseInt(localStorage.getItem("commandSeq") || "0", 10) + 1;
    localStorage.setItem("commandSeq", String(seq));
    return seq;
}

// Sync the local command sequence counter to the firmware's last accepted value.
// Only acts when the UI's stored public key matches the firmware's reported key —
// a monitoring-only client watching a different aircraft is left untouched.
// Called from both the telemetry parser (lseq field) and the ack parser.
function syncCommandSeqIfAuthorized(firmwareSeq) {
    if (isNaN(firmwareSeq)) return;
    var uiPubKey = localStorage.getItem("commandPublicKeyBase64");
    if (!uiPubKey || !data.firmwarePublicKey || uiPubKey !== data.firmwarePublicKey) return;
    var currentSeq = parseInt(localStorage.getItem("commandSeq") || "0", 10);
    if (currentSeq <= firmwareSeq) {
        // getNextCommandSeq() adds 1 before use, so storing firmwareSeq here
        // means the next command will use firmwareSeq+1, which the firmware accepts.
        localStorage.setItem("commandSeq", String(firmwareSeq));
    }
}

async function signCanonical(canonical) {
    var privateKeyJwk = localStorage.getItem("commandPrivateKey");
    if (!privateKeyJwk) throw new Error("No private key in localStorage");
    var privateKey = await window.crypto.subtle.importKey(
        "jwk",
        JSON.parse(privateKeyJwk),
        { name: "Ed25519" },
        false,
        ["sign"]
    );
    var sigBytes = await window.crypto.subtle.sign(
        { name: "Ed25519" },
        privateKey,
        new TextEncoder().encode(canonical)
    );
    var bytes = new Uint8Array(sigBytes);
    var binary = "";
    for (var i = 0; i < bytes.length; i++) binary += String.fromCharCode(bytes[i]);
    return btoa(binary);
}

// state:       null for stateless commands (ping), 0 or 1 for RC channel commands.
// extraFields: optional object of additional key:value pairs appended to the
//              payload but NOT part of the signed canonical (same as state).
//              Example: { heading: 270 }
// historyLabel: optional string override for the command history display.
//
// Extra fields are included in the payload but NOT in the signed canonical — the
// firmware verifies only cmd/cid/seq and reads the extra fields separately.
export async function publishCommand(cmdType, state = null, extraFields = null, historyLabel = null) {
    if (!mqttConnected || !commandTopic) return null;
    if (!localStorage.getItem("commandPrivateKey")) {
        console.warn("publishCommand: no private key configured — command not sent");
        return null;
    }
    var cid = generateCid();
    var seq = getNextCommandSeq();
    var canonical = "cmd:" + cmdType + ",cid:" + cid + ",seq:" + seq;
    var sig;
    try {
        sig = await signCanonical(canonical);
    } catch (e) {
        console.error("publishCommand: signing failed:", e);
        return null;
    }
    var payload = state !== null
        ? "cmd:" + cmdType + ",state:" + state + ",cid:" + cid + ",seq:" + seq + ",sig:" + sig + ","
        : canonical + ",sig:" + sig + ",";
    if (extraFields !== null) {
        for (var k in extraFields) {
            payload += k + ":" + extraFields[k] + ",";
        }
    }
    var historyType = historyLabel !== null ? historyLabel
        : state !== null ? cmdType + ":" + (state ? "ON" : "OFF") : cmdType;
    var entry = { cid: cid, type: historyType, timestamp: Date.now(), status: 'sent' };
    pendingCommands[cid] = { type: cmdType, messageCount: 0 };
    commandHistory.unshift(entry);
    var message = new Paho.MQTT.Message(payload);
    message.destinationName = commandTopic;
    message.qos = 0;
    mqtt.send(message);
    console.log("Command sent: " + payload);
    return cid;
}

// ── Mission download buffer ───────────────────────────────────────────────────
var missionDownloadBuffer = [];
export function clearMissionDownloadBuffer() { missionDownloadBuffer = []; }
export function getMissionDownloadBuffer()   { return missionDownloadBuffer; }

function parseMissionDownloadMessage(payload) {
    var parts = payload.split(',');
    var dlwp = null, la = null, lo = null, al = null, ac = 1, p1 = 0, p2 = 0, p3 = 0, f = 0;
    for (var i = 0; i < parts.length; i++) {
        var kv = parts[i].split(':');
        if (kv.length < 2) continue;
        switch (kv[0]) {
            case 'dlwp': dlwp = parseInt(kv[1]); break;
            case 'la':   la   = parseInt(kv[1]); break;
            case 'lo':   lo   = parseInt(kv[1]); break;
            case 'al':   al   = parseInt(kv[1]); break;
            case 'ac':   ac   = parseInt(kv[1]); break;
            case 'p1':   p1   = parseInt(kv[1]); break;
            case 'p2':   p2   = parseInt(kv[1]); break;
            case 'p3':   p3   = parseInt(kv[1]); break;
            case 'f':    f    = parseInt(kv[1]); break;
        }
    }
    if (dlwp === null || la === null || lo === null || al === null) return;
    missionDownloadBuffer.push({ dlwp, la, lo, al, ac, p1, p2, p3, f });
}

// Callbacks registered by waitForCommandAck; keyed by cid.
// Each entry is { resolve, reject }.
const commandAckCallbacks = {};

// Returns a Promise that resolves when the firmware ACKs the given cid,
// or rejects on NACK or after timeoutMs milliseconds.
export function waitForCommandAck(cid, timeoutMs = 15000) {
    return new Promise((resolve, reject) => {
        commandAckCallbacks[cid] = { resolve, reject };
        setTimeout(() => {
            if (commandAckCallbacks[cid]) {
                delete commandAckCallbacks[cid];
                reject(new Error('timeout'));
            }
        }, timeoutMs);
    });
}

function resolveCommandAck(cid) {
    if (pendingCommands[cid]) {
        delete pendingCommands[cid];
        for (var i = 0; i < commandHistory.length; i++) {
            if (commandHistory[i].cid === cid) {
                commandHistory[i].status = 'received';
                break;
            }
        }
    }
    if (commandAckCallbacks[cid]) {
        commandAckCallbacks[cid].resolve();
        delete commandAckCallbacks[cid];
    }
    console.log('Command acknowledged: ' + cid);
}

function checkCommandTimeouts() {
    for (var cid in pendingCommands) {
        pendingCommands[cid].messageCount++;
        if (pendingCommands[cid].messageCount >= 10) {
            delete pendingCommands[cid];
            for (var i = 0; i < commandHistory.length; i++) {
                if (commandHistory[i].cid === cid) {
                    commandHistory[i].status = 'lost';
                    break;
                }
            }
            console.log('Command lost (no ack after 10 messages): ' + cid);
        }
    }
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
        var msgTopic = message.destinationName;
        var payload = message.payloadString;

        if (otherAircraft[msgTopic]) {
            // Secondary aircraft — parse silently, do not log to session
            parseSecondaryTelemetry(msgTopic, payload);
        } else {
            // Primary aircraft
            var line = new Date().getTime().toString() + '|' + payload;
            mqttlog.push(line);
            lastMessageDate = new Date();
            console.log("msg: " + payload);
            parseTelemetryData(payload);  // may resolve a pending cid
            checkCommandTimeouts();
            if (onMessageCallback) onMessageCallback(line);
        }
    }
};

// MQTTconnect();

// ─── Multi-aircraft monitoring ────────────────────────────────────────────────

// Colour palette for secondary aircraft. Each entry provides a CSS hue-rotate
// angle and a hex colour string used for MapLibre GL JS line layers and the
// label border. The filter chain in MapScripts.js normalises any source icon
// to white via brightness(0)+invert(1), then sepia gives a starting hue of
// ~38°. Final hue ≈ 38° + N, so N = target hue − 38°.
var SECONDARY_PALETTE = [
    { hueRotate: 322, color: '#F66' },   // red/pink    (target   0°: 0−38 → 322)
    { hueRotate: 352, color: '#FA4' },   // orange      (target  30°: 30−38 → 352)
    { hueRotate: 142, color: '#4EE' },   // cyan        (target 180°: 180−38 → 142)
    { hueRotate: 22,  color: '#EE4' },   // yellow      (target  60°: 60−38 → 22)
    { hueRotate: 232, color: '#84F' },   // purple      (target 270°: 270−38 → 232)
    { hueRotate: 82,  color: '#AFA' },   // light green (target 120°: 120−38 → 82)
];
var paletteNextIndex = 0;

// keyed by MQTT topic string.
// Shape: { topic, callsign, lat, lon, alt, gsp, vsp, course, lastSeen, colour, hueRotate, flightPath[] }
export var otherAircraft = {};

export function addMonitoredTopic(newTopic) {
    var primary    = localStorage.getItem("mqttTopic") || "";
    var primaryCmd = localStorage.getItem("mqttCommandTopic") || "";
    if (!newTopic)                                  return "Topic cannot be empty.";
    if (newTopic === primary || newTopic === primaryCmd)
                                                    return "Topic conflicts with the primary aircraft topic.";
    if (otherAircraft[newTopic])                    return "Topic is already monitored.";

    var palette = SECONDARY_PALETTE[paletteNextIndex % SECONDARY_PALETTE.length];
    paletteNextIndex++;

    otherAircraft[newTopic] = {
        topic:          newTopic,
        callsign:       "",
        asl:            null,
        heading:        0,
        batteryPercent: null,
        flightTime:     null,
        homeDistance:   null,
        lat:        0,   // integer degrees × 1e7
        lon:        0,
        alt:        0,   // integer cm
        gsp:        0,   // integer cm/s
        vsp:        0,   // integer cm/s
        course:     0,   // integer degrees true
        lastSeen:   0,   // Date.now() ms
        colour:     palette.color,
        hueRotate:  palette.hueRotate,
        flightPath: [],  // [[lon_deg, lat_deg], ...]
    };

    saveMonitoredTopics();
    if (mqttConnected && mqtt) mqtt.subscribe(newTopic, { qos: 0 });
    return null; // no error
}

export function removeMonitoredTopic(topicToRemove) {
    if (!otherAircraft[topicToRemove]) return;
    delete otherAircraft[topicToRemove];
    saveMonitoredTopics();
    if (mqttConnected && mqtt) {
        try { mqtt.unsubscribe(topicToRemove); } catch(e) {}
    }
}

function saveMonitoredTopics() {
    localStorage.setItem("gcssMonitoredTopics", JSON.stringify(Object.keys(otherAircraft)));
}

// Restores the secondary topic list from localStorage and subscribes.
// Call once from DOMContentLoaded (after MQTTconnect is called).
export function loadAndSubscribeMonitoredTopics() {
    var stored = localStorage.getItem("gcssMonitoredTopics");
    if (!stored) return;
    try {
        var topics = JSON.parse(stored);
        for (var i = 0; i < topics.length; i++) {
            if (!otherAircraft[topics[i]]) addMonitoredTopic(topics[i]);
        }
    } catch(e) { console.error("loadAndSubscribeMonitoredTopics:", e); }
}

function parseSecondaryTelemetry(incomingTopic, payload) {
    var entry = otherAircraft[incomingTopic];
    if (!entry) return;

    var arrPayload = payload.split(",");
    var rawGla = null, rawGlo = null;

    for (var i = 0; i < arrPayload.length; i++) {
        if (!arrPayload[i]) continue;
        var parts = arrPayload[i].split(":");
        if (parts.length < 2) continue;
        var raw;

        switch (parts[0]) {
            case "cs":
                if (/^[A-Za-z0-9_-]+$/.test(parts[1]) && parts[1].length <= 16)
                    entry.callsign = parts[1];
                break;
            case "gla":
                raw = parseInt(parts[1]);
                if (inRange(raw, -900000000, 900000000)) rawGla = raw;
                break;
            case "glo":
                raw = parseInt(parts[1]);
                if (inRange(raw, -1800000000, 1800000000)) rawGlo = raw;
                break;
            case "alt":
                raw = parseInt(parts[1]);
                if (inRange(raw, -1000000, 10000000)) entry.alt = raw;
                break;
            case "asl":
                raw = parseFloat(parts[1]);
                if (inRange(raw, -500, 20000)) entry.asl = raw;
                break;
            case "hea":
                raw = parseInt(parts[1]);
                if (inRange(raw, 0, 360)) entry.heading = raw;
                break;
            case "bfp":
                raw = parseInt(parts[1]);
                if (inRange(raw, 0, 100)) entry.batteryPercent = raw;
                break;
            case "flt":
                raw = parseInt(parts[1]);
                if (raw >= 0) entry.flightTime = raw;
                break;
            case "hds":
                raw = parseInt(parts[1]);
                if (raw >= 0) entry.homeDistance = raw;
                break;
            case "gsp":
                raw = parseInt(parts[1]);
                if (inRange(raw, 0, 15000)) entry.gsp = raw;
                break;
            case "vsp":
                raw = parseInt(parts[1]);
                if (inRange(raw, -60000, 60000)) entry.vsp = raw;
                break;
            case "ggc":
                raw = parseInt(parts[1]);
                if (inRange(raw, 0, 359)) entry.course = raw;
                break;
        }
    }

    if (rawGla !== null && rawGlo !== null) {
        entry.lat = rawGla;
        entry.lon = rawGlo;
        entry.lastSeen = Date.now();
        entry.flightPath.push([rawGlo / 10000000.0, rawGla / 10000000.0]);
    }
}

// Dev / screenshot injection — call from console or Playwright.
// Accepts a single payload, a single log line (timestamp|payload),
// or a multi-line string of either format — each line is processed in order.
//   gcssInject("ran:25,hea:351,gla:-234816623,glo:-471517217,...")
//   gcssInject("1702818853162|ran:25,hea:351,...\n1702818854089|ran:21,...")
window.gcssInject = function(input) {
    var lines = input.split('\n');
    for (var i = 0; i < lines.length; i++) {
        var line = lines[i].trim();
        if (!line) continue;
        var payload = line.includes('|') ? line.slice(line.indexOf('|') + 1) : line;
        parseTelemetryData(payload);
    }
    checkCommandTimeouts();
};

export let data = {};
// _gcssData is a live getter so it always reflects the current data reference,
// even after resetDataObject() replaces it. Use from console or Playwright.
Object.defineProperty(window, '_gcssData', { get: () => data, configurable: true });
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
        protocolVersion: 1, // 1 = current/legacy; set from low-priority message (pv field)
        downlinkStatus: 0, // 0 = firmware not subscribed to command topic, 1 = subscribed ok
        mspRcOverride: 0, // 0 = MSP RC Override flight mode not active, 1 = active (commands can be sent)
        cmdRth: 0,     // 1 = firmware actively overriding this channel, 0 = not overriding
        cmdAltHold: 0,
        cmdCruise: 0,
        cmdBeeper: 0,
        cmdWp: 0,
        cmdPosHold: 0,
        fmCruise: 0,   // 1 = Cruise/Course Hold flight mode active (any source)
        fmAltHold: 0,  // 1 = Altitude Hold flight mode active (any source)
        fmWp: 0,       // 1 = WP/Mission flight mode active (any source)
        fmPosHold: 0,  // 1 = Position Hold flight mode active (any source)
        maxWaypoints: 0, // max WPs supported by FC (from wpmax field); 0 = not yet received
        fcVersion: "",       // FC firmware version string, e.g. "9.0.1" (empty = not yet received)
        extCmdsSupported: 0, // computed from fcVersion: 0 = none; >= 1 = extended MSP commands supported
        firmwarePublicKey: "", // base64-encoded Ed25519 public key from firmware (empty = not yet received)
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
            gpsGroundCourse: 0,
        },
        lastMessage: {
            rollAngle: 0,
            pitchAngle: 0,
            heading: 0,
            altitude: 0,
            groundSpeed: 0,
            verticalSpeed: 0,
            gpsGroundCourse: 0,
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

    // Snapshot current smooth positions as the interpolation start point for all EFIS fields.
    // This must happen before any field is updated so that fields absent from this message
    // animate from where they currently are (no restart), while fields present in this message
    // animate from their current smooth position to the new value.
    data.lastMessage.rollAngle      = data.estimations.rollAngle;
    data.lastMessage.pitchAngle     = data.estimations.pitchAngle;
    data.lastMessage.heading        = data.estimations.heading;
    data.lastMessage.altitude       = data.estimations.altitude;
    data.lastMessage.groundSpeed    = data.estimations.groundSpeed;
    data.lastMessage.verticalSpeed  = data.estimations.verticalSpeed;
    data.lastMessage.gpsGroundCourse = data.estimations.gpsGroundCourse;

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
                    data.rollAngle = raw / 10.0;
                }
                break;
            case "pan":
                raw = parseFloat(arrData[1]);
                if(inRange(raw, -900, 900)) {
                    data.pitchAngle = raw / 10.0;
                }
                break;
            case "hea":
                raw = parseFloat(arrData[1]);
                if(inRange(raw, 0, 359)) {
                    data.heading = raw;
                }
                break;
            case "alt":
                raw = parseInt(arrData[1]);
                if(inRange(raw, -1000000, 10000000)) {
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
                    data.groundSpeed = raw;
                }
                break;
            case "vsp":
                raw = parseInt(arrData[1]);
                if(inRange(raw, -60000, 60000)) {
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
            case "dls":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1)
                    data.downlinkStatus = raw;
                break;
            case "mro":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1)
                    data.mspRcOverride = raw;
                break;
            case "cmdrth":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.cmdRth = raw;
                break;
            case "cmdalt":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.cmdAltHold = raw;
                break;
            case "cmdcrs":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.cmdCruise = raw;
                break;
            case "cmdbep":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.cmdBeeper = raw;
                break;
            case "cmdwp":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.cmdWp = raw;
                break;
            case "fmcrs":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.fmCruise = raw;
                break;
            case "fmalt":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.fmAltHold = raw;
                break;
            case "fmwp":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.fmWp = raw;
                break;
            case "cmdph":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.cmdPosHold = raw;
                break;
            case "fmph":
                raw = parseInt(arrData[1]);
                if(raw === 0 || raw === 1) data.fmPosHold = raw;
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
            case "pv":
                raw = parseInt(arrData[1]);
                if(inRange(raw, 1, 999)) {
                    data.protocolVersion = raw;
                    // console.log("Protocol version: " + raw);
                }
                break;
            case "pk":
                // Base64-encoded Ed25519 public key (44 chars: 43 base64 chars + 1 '=' padding)
                if (/^[A-Za-z0-9+/]{43}=$/.test(arrData[1]))
                    data.firmwarePublicKey = arrData[1];
                break;
            case "lseq":
                // Last accepted command sequence number — sync our counter if authorized.
                syncCommandSeqIfAuthorized(parseInt(arrData[1], 10));
                break;
            case "fcver":
                if (/^\d+\.\d+\.\d+$/.test(arrData[1])) {
                    data.fcVersion = arrData[1];
                    var parts = arrData[1].split('.').map(Number);
                    var major = parts[0], minor = parts[1], patch = parts[2];
                    // Extended commands (setheading, setalt, jumpwp) require FC version > 9.0.1
                    data.extCmdsSupported = (major > 9 ||
                        (major === 9 && minor > 0) ||
                        (major === 9 && minor === 0 && patch > 1)) ? 1 : 0;
                }
                break;
            case "wpmax":
                raw = parseInt(arrData[1]);
                if (inRange(raw, 1, 255))
                    data.maxWaypoints = raw;
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

function parseCommandMessage(payload) {
    var arrPayload = payload.split(",");
    var cmd = null;
    var cid = null;
    var lseq = NaN;

    for (var i = 0; i < arrPayload.length; i++) {
        var arrData = arrPayload[i].split(":");
        if (arrData.length < 2) continue;
        if (arrData[0] === "cmd")  cmd  = arrData[1];
        if (arrData[0] === "cid")  cid  = arrData[1];
        if (arrData[0] === "lseq") lseq = parseInt(arrData[1], 10);
    }

    if (cmd === "ack" && cid) {
        resolveCommandAck(cid);
        // Ack carries the new lastSeq — sync all connected UIs immediately.
        syncCommandSeqIfAuthorized(lseq);
    }
    if (cmd === "nack" && cid) {
        var reason = '';
        for (var j = 0; j < arrPayload.length; j++) {
            var kv = arrPayload[j].split(":");
            if (kv[0] === "reason") { reason = kv[1] || ''; break; }
        }
        if (commandAckCallbacks[cid]) {
            commandAckCallbacks[cid].reject(new Error(reason || 'nack'));
            delete commandAckCallbacks[cid];
        }
        for (var i = 0; i < commandHistory.length; i++) {
            if (commandHistory[i].cid === cid) {
                commandHistory[i].status = 'lost';
                break;
            }
        }
        if (pendingCommands[cid]) delete pendingCommands[cid];
        console.log('Command NACK received: ' + cid + ' reason: ' + reason);
    }
}

export function parseTelemetryData(payload) {
    var arrPayload = payload.split(",");

    if(arrPayload.length > 0) {
        if(payload.startsWith("dlwp:"))
            parseMissionDownloadMessage(payload);
        else if(payload.startsWith("wpno:"))
            parseWaypointMessage(payload);
        else if(payload.startsWith("cmd:"))
            parseCommandMessage(payload);
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
        
        data.lastMessage.altitude        = data.altitude;
        data.lastMessage.rollAngle       = data.rollAngle;
        data.lastMessage.pitchAngle      = data.pitchAngle;
        data.lastMessage.groundSpeed     = data.groundSpeed;
        data.lastMessage.verticalSpeed   = data.verticalSpeed;
        data.lastMessage.heading         = data.heading;
        data.lastMessage.gpsGroundCourse = data.gpsGroundCourse;

        data.estimations.altitude        = data.altitude;
        data.estimations.rollAngle       = data.rollAngle;
        data.estimations.pitchAngle      = data.pitchAngle;
        data.estimations.groundSpeed     = data.groundSpeed;
        data.estimations.verticalSpeed   = data.verticalSpeed;
        data.estimations.heading         = data.heading;
        data.estimations.gpsGroundCourse = data.gpsGroundCourse;
        return;
    }

    // This is the percentage of the movement to the last frame
    var movePercent = (timeSinceLastFrame / pageSettings.messageInterval) * 100;

    data.estimations.altitude        = rangeNumbers(movePercent, 0, 100, data.lastMessage.altitude, data.altitude);
    data.estimations.rollAngle       = rangeNumbers(movePercent, 0, 100, data.lastMessage.rollAngle, data.rollAngle);
    data.estimations.pitchAngle      = rangeNumbers(movePercent, 0, 100, data.lastMessage.pitchAngle, data.pitchAngle);
    data.estimations.groundSpeed     = rangeNumbers(movePercent, 0, 100, data.lastMessage.groundSpeed, data.groundSpeed);
    data.estimations.verticalSpeed   = rangeNumbers(movePercent, 0, 100, data.lastMessage.verticalSpeed, data.verticalSpeed);
    data.estimations.heading         = rangeNumbers360(movePercent, 0, 100, data.lastMessage.heading, data.heading);
    data.estimations.gpsGroundCourse = rangeNumbers360(movePercent, 0, 100, data.lastMessage.gpsGroundCourse, data.gpsGroundCourse);


}

// Timing and refresh intervals (consumed by estimateEfis and PageScripts timers)
export let pageSettings = {
    efisRefreshInterval: 50,
    mapFastRefreshInterval: 100,
    mapAndDataRefreshInterval: 500,
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
