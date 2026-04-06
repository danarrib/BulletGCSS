// bsPageScripts.js — entry point for bsui.html (Bootstrap migration)
// Handles session management, MQTT, panel wiring, and the render timer loop.

import { data, mqtt, mqttConnected, MQTTconnect, MQTTSetDefaultSettings,
         savemqttlog, replaymqttlog, stopreplaymqttlog, resetDataObject,
         pageSettings, estimateEfis, estimatePosition,
         updatingWpAltitudes, setOnMessageCallback, setOnReplayStop,
         restoreFromSessionMessages, replayFromSessionMessages,
         publishCommand, commandHistory, otherAircraft,
         addMonitoredTopic, removeMonitoredTopic,
         loadAndSubscribeMonitoredTopics, secondsToNiceTime,
         getDistanceBetweenTwoPoints } from './CommScripts.js';
import { renderEFIS } from './EfisScripts.js';
import { drawAircraftOnMap, drawAircraftPathOnMap, drawCourseLineOnMap, drawMissionOnMap,
         drawHomeOnMap, drawUserOnMap, centerMap, getMissionWaypointsAltitude,
         getUserLocation, startOrientationTracking, user_moved_map, setUserMovedMap,
         setMapStyle, setOnWaypointClick, updateSecondaryAircraftOnMap,
         removeSecondaryAircraftFromMap, setOnSecondaryAircraftClick } from './MapScripts.js';
import { updateDataView, setUIUnits, toggleBlinkFast, toggleBlinkSlow,
         openGoogleMaps } from './InfoPanelScripts.js';
import { openDB, createSession, closeSession, getOpenSession, listSessions,
         getSessionMessages, countSessionMessages, appendMessage,
         deleteSession, renameSession } from './SessionScripts.js';
import { openMissionPlanner, initMissionPlannerButtons } from './MissionPlannerScripts.js';

// openGoogleMaps is called from dynamically-generated HTML in InfoPanelScripts.js
window.openGoogleMaps = openGoogleMaps;

var currentSessionId = null;
var currentPopupTopic = null;
var noSleep = new NoSleep();

// ── Default settings ──────────────────────────────────────────────────────────

function checkForDefaultSettings() {
    if (localStorage.getItem("mqttHost") === null)
        localStorage.setItem("mqttHost", "broker.emqx.io");
    if (localStorage.getItem("mqttPort") === null)
        localStorage.setItem("mqttPort", "8084");
    if (localStorage.getItem("mqttTopic") === null)
        localStorage.setItem("mqttTopic", "bulletgcss/telem/your_callsign");
    if (localStorage.getItem("mqttCommandTopic") === null)
        localStorage.setItem("mqttCommandTopic", "bulletgcss/cmd/your_callsign");
    if (localStorage.getItem("mqttUseTLS") === null)
        localStorage.setItem("mqttUseTLS", "true");
    if (localStorage.getItem("ui_speed") === null)
        localStorage.setItem("ui_speed", "kmh");
    if (localStorage.getItem("ui_distance") === null)
        localStorage.setItem("ui_distance", "km");
    if (localStorage.getItem("ui_altitude") === null)
        localStorage.setItem("ui_altitude", "m");
    if (localStorage.getItem("ui_current") === null)
        localStorage.setItem("ui_current", "a");
    if (localStorage.getItem("ui_capacity") === null)
        localStorage.setItem("ui_capacity", "mah");
    if (localStorage.getItem("ui_efficiency") === null)
        localStorage.setItem("ui_efficiency", "mahkm");
    if (localStorage.getItem("ui_map_style") === null)
        localStorage.setItem("ui_map_style", "liberty");
    if (localStorage.getItem("ui_elevation_provider") === null) {
        if (location.href.indexOf("bulletgcss.outros.net") > 0)
            localStorage.setItem("ui_elevation_provider", "OpenTopoDataDirect");
        else
            localStorage.setItem("ui_elevation_provider", "OpenTopoData");
    }
}

checkForDefaultSettings();

// ── RC command table ──────────────────────────────────────────────────────────

var rcCommands = [
    { cmd: "rth",     onId: "btRthOn",     offId: "btRthOff",     dataKey: "cmdRth"     },
    { cmd: "althold", onId: "btAltHoldOn", offId: "btAltHoldOff", dataKey: "cmdAltHold" },
    { cmd: "cruise",  onId: "btCruiseOn",  offId: "btCruiseOff",  dataKey: "cmdCruise"  },
    { cmd: "beeper",  onId: "btBeeperOn",  offId: "btBeeperOff",  dataKey: "cmdBeeper"  },
    { cmd: "wp",      onId: "btWpOn",      offId: "btWpOff",      dataKey: "cmdWp",
      onCondition: function() { return data.isWaypointMissionValid === 1; } },
    { cmd: "poshold", onId: "btPosHoldOn", offId: "btPosHoldOff", dataKey: "cmdPosHold" },
];

// ── Commands panel ────────────────────────────────────────────────────────────

function openCommandsModal() {
    updateCommandsPanel();
    bootstrap.Modal.getOrCreateInstance(document.getElementById('bsModalCommands')).show();
}

function updateCommandsPanel() {
    var downlinkOk = mqttConnected && data.downlinkStatus === 1;
    var rcOk = downlinkOk && data.mspRcOverride === 1;

    document.getElementById("commandsDownlinkWarning").style.display = downlinkOk ? "none" : "block";
    document.getElementById("commandsMroWarning").style.display = (downlinkOk && !rcOk) ? "block" : "none";

    document.getElementById("btSendPing").disabled = !downlinkOk;
    var extOk = data.extCmdsSupported >= 1;
    document.getElementById("btSendHeading").disabled  = !(downlinkOk && extOk && data.fmCruise === 1);
    document.getElementById("btSendJumpWp").disabled   = !(downlinkOk && extOk && data.fmWp === 1);
    document.getElementById("btSendAltitude").disabled = !(downlinkOk && extOk && data.fmAltHold === 1);

    var wpInput = document.getElementById("inputWpIndex");
    var maxWp = data.waypointCount > 0 ? data.waypointCount : 255;
    wpInput.max = maxWp;
    wpInput.placeholder = "1\u2013" + maxWp;

    for (var i = 0; i < rcCommands.length; i++) {
        var entry  = rcCommands[i];
        var onBtn  = document.getElementById(entry.onId);
        var offBtn = document.getElementById(entry.offId);

        var modeActive = rcOk && data[entry.dataKey] === 1;
        var onAllowed  = rcOk && (!entry.onCondition || entry.onCondition());

        onBtn.disabled  = !onAllowed;
        offBtn.disabled = !modeActive;

        onBtn.classList.toggle('btn-active', modeActive);
        onBtn.classList.remove('btn-inactive');
        offBtn.classList.remove('btn-active', 'btn-inactive');
    }

    renderCommandHistory();
}

function renderCommandHistory() {
    var container = document.getElementById("commandsList");
    if (commandHistory.length === 0) {
        container.innerHTML = '<p class="text-secondary small">No commands sent yet.</p>';
        return;
    }
    var html = '';
    for (var i = 0; i < commandHistory.length; i++) {
        var cmd = commandHistory[i];
        var time = new Date(cmd.timestamp).toLocaleTimeString();
        var statusClass = 'cmd-status-' + cmd.status;
        var statusLabel = cmd.status.charAt(0).toUpperCase() + cmd.status.slice(1);
        html += '<div class="cmd-item">';
        html += '<div class="cmd-info">';
        html += '<span class="cmd-type">' + cmd.type + '</span>';
        html += '<span class="cmd-meta">' + time + ' &bull; ID: ' + cmd.cid + '</span>';
        html += '</div>';
        html += '<span class="cmd-status ' + statusClass + '">' + statusLabel + '</span>';
        html += '</div>';
    }
    container.innerHTML = html;
}

// ── Broker settings ───────────────────────────────────────────────────────────

function openBrokerSettings() {
    document.getElementById("brokerHost").value  = localStorage.getItem("mqttHost") || "";
    document.getElementById("brokerPort").value  = localStorage.getItem("mqttPort") || "";
    document.getElementById("brokerUser").value  = localStorage.getItem("mqttUser") || "";
    document.getElementById("brokerPass").value  = localStorage.getItem("mqttPass") || "";
    document.getElementById("brokerTopic").value        = localStorage.getItem("mqttTopic") || "";
    document.getElementById("brokerCommandTopic").value = localStorage.getItem("mqttCommandTopic") || "";
    document.getElementById("brokerUseTLS").checked = (localStorage.getItem("mqttUseTLS") === "true");
    bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('bsOffcanvasSettings')).hide();
    bootstrap.Modal.getOrCreateInstance(document.getElementById('bsModalBroker')).show();
}

function saveBrokerSettings() {
    localStorage.setItem("mqttHost",  document.getElementById("brokerHost").value);
    localStorage.setItem("mqttPort",  document.getElementById("brokerPort").value);

    var brokerUser = document.getElementById("brokerUser").value;
    if (brokerUser.length > 0)
        localStorage.setItem("mqttUser", brokerUser);
    else
        localStorage.removeItem("mqttUser");

    var brokerPass = document.getElementById("brokerPass").value;
    if (brokerUser.length > 0)
        localStorage.setItem("mqttPass", brokerPass);
    else
        localStorage.removeItem("mqttPass");

    localStorage.setItem("mqttTopic",        document.getElementById("brokerTopic").value);
    localStorage.setItem("mqttCommandTopic", document.getElementById("brokerCommandTopic").value);
    localStorage.setItem("mqttUseTLS",       document.getElementById("brokerUseTLS").checked ? "true" : "false");

    if (mqttConnected) mqtt.disconnect();
    bootstrap.Modal.getOrCreateInstance(document.getElementById('bsModalBroker')).hide();
}

function resetBrokerSettings() {
    MQTTSetDefaultSettings();
    if (mqttConnected) mqtt.disconnect();
    bootstrap.Modal.getOrCreateInstance(document.getElementById('bsModalBroker')).hide();
}

// ── UI settings ───────────────────────────────────────────────────────────────

function openUISettings() {
    document.getElementById("ui_speed").value              = localStorage.getItem("ui_speed") || "kmh";
    document.getElementById("ui_distance").value           = localStorage.getItem("ui_distance") || "km";
    document.getElementById("ui_altitude").value           = localStorage.getItem("ui_altitude") || "m";
    document.getElementById("ui_current").value            = localStorage.getItem("ui_current") || "a";
    document.getElementById("ui_capacity").value           = localStorage.getItem("ui_capacity") || "mah";
    document.getElementById("ui_efficiency").value         = localStorage.getItem("ui_efficiency") || "mahkm";
    document.getElementById("ui_map_style").value          = localStorage.getItem("ui_map_style") || "liberty";
    document.getElementById("ui_elevation_provider").value = localStorage.getItem("ui_elevation_provider") || "OpenTopoData";
    bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('bsOffcanvasSettings')).hide();
    bootstrap.Modal.getOrCreateInstance(document.getElementById('bsModalUI')).show();
}

function saveUISettings() {
    localStorage.setItem("ui_speed",              document.getElementById("ui_speed").value);
    localStorage.setItem("ui_distance",           document.getElementById("ui_distance").value);
    localStorage.setItem("ui_altitude",           document.getElementById("ui_altitude").value);
    localStorage.setItem("ui_current",            document.getElementById("ui_current").value);
    localStorage.setItem("ui_capacity",           document.getElementById("ui_capacity").value);
    localStorage.setItem("ui_efficiency",         document.getElementById("ui_efficiency").value);
    localStorage.setItem("ui_elevation_provider", document.getElementById("ui_elevation_provider").value);
    setMapStyle(document.getElementById("ui_map_style").value);
    setUIUnits();
    bootstrap.Modal.getOrCreateInstance(document.getElementById('bsModalUI')).hide();
}

// ── Security panel ────────────────────────────────────────────────────────────

function openSecurityModal() {
    loadSecurityPanel();
    bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('bsOffcanvasSettings')).hide();
    bootstrap.Modal.getOrCreateInstance(document.getElementById('bsModalSecurity')).show();
}

function loadSecurityPanel() {
    var hasKey   = localStorage.getItem("commandPrivateKey") !== null;
    var uiBase64 = localStorage.getItem("commandPublicKeyBase64");

    // Migration: derive base64 from hex if key was generated before base64 storage was added
    if (hasKey && !uiBase64) {
        var hexStr = localStorage.getItem("commandPublicKeyHex");
        if (hexStr) {
            var hexBytes = hexStr.split(',').map(function(h) { return parseInt(h.trim(), 16); });
            if (hexBytes.length === 32) {
                uiBase64 = btoa(String.fromCharCode.apply(null, hexBytes));
                localStorage.setItem("commandPublicKeyBase64", uiBase64);
            }
        }
    }

    var fwBase64  = data.firmwarePublicKey;
    var allZeros  = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=";
    var statusEl  = document.getElementById("securityKeyStatus");
    var statusText, statusColor;

    if (!hasKey && (!fwBase64 || fwBase64 === allZeros)) {
        statusText  = "No key pair configured. Generate one below.";
        statusColor = "#aaa";
    } else if (!hasKey && fwBase64 && fwBase64 !== allZeros) {
        statusText  = "\u26a0 Firmware has a key but the UI key is missing (localStorage cleared?). Regenerate and re-flash.";
        statusColor = "#f90";
    } else if (hasKey && !fwBase64) {
        statusText  = "Key pair ready. Waiting for firmware telemetry\u2026";
        statusColor = "#aaa";
    } else if (hasKey && fwBase64 === allZeros) {
        statusText  = "\u26a0 Firmware has no key. Paste the public key into Config.h and re-flash.";
        statusColor = "#f90";
    } else if (hasKey && fwBase64 === uiBase64) {
        statusText  = "\u2713 Keys match. Firmware is ready for signed commands.";
        statusColor = "#3f3";
    } else {
        statusText  = "\u2717 Key mismatch! Re-flash the firmware with the current public key.";
        statusColor = "#f44";
    }
    statusEl.textContent = statusText;
    statusEl.style.color = statusColor;

    var pubKeyHex = localStorage.getItem("commandPublicKeyHex");
    document.getElementById("securityPublicKeyArea").value = hasKey && pubKeyHex
        ? "const uint8_t commandPublicKey[32] = { " + pubKeyHex + " };"
        : "";
    document.getElementById("btCopyPublicKey").style.display  = hasKey ? "inline-block" : "none";
    document.getElementById("securityKeyHint").style.display  = hasKey ? "block" : "none";
    document.getElementById("btGenerateKey").value = hasKey ? "Regenerate key pair" : "Generate key pair";
}

async function generateKeyPair() {
    var hasKey = localStorage.getItem("commandPrivateKey") !== null;
    if (hasKey) {
        if (!confirm("Generating a new key pair will require re-flashing the firmware with the new public key. The existing key pair will be lost.\n\nContinue?"))
            return;
    }
    try {
        var keyPair = await window.crypto.subtle.generateKey({ name: "Ed25519" }, true, ["sign", "verify"]);
        var privateKeyJwk = await window.crypto.subtle.exportKey("jwk", keyPair.privateKey);
        localStorage.setItem("commandPrivateKey", JSON.stringify(privateKeyJwk));

        var publicKeyRaw = await window.crypto.subtle.exportKey("raw", keyPair.publicKey);
        var bytes = new Uint8Array(publicKeyRaw);
        var hexArray = Array.from(bytes).map(function(b) {
            return '0x' + b.toString(16).padStart(2, '0');
        }).join(', ');
        localStorage.setItem("commandPublicKeyHex", hexArray);

        var base64Key = btoa(String.fromCharCode.apply(null, bytes));
        localStorage.setItem("commandPublicKeyBase64", base64Key);

        loadSecurityPanel();
    } catch(e) {
        alert("Error generating key pair: " + e.message + "\n\nYour browser may not support Ed25519. Please use Chrome 113+, Firefox 130+, or Safari 17+.");
    }
}

async function copyPublicKey() {
    var text = document.getElementById("securityPublicKeyArea").value;
    if (!text) return;
    try {
        await navigator.clipboard.writeText(text);
        document.getElementById("btCopyPublicKey").value = "Copied!";
        setTimeout(function() {
            document.getElementById("btCopyPublicKey").value = "Copy to clipboard";
        }, 2000);
    } catch(e) {
        document.getElementById("securityPublicKeyArea").select();
    }
}

// ── Sessions panel ────────────────────────────────────────────────────────────

function openSessionsPanel() {
    renderSessionsList().then(function() {
        bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('bsOffcanvasSessions')).show();
    });
}

async function renameCurrentSession() {
    if (currentSessionId === null) return;
    var newName = document.getElementById("currentSessionName").value.trim();
    if (!newName) return;
    await renameSession(currentSessionId, newName);
    await renderSessionsList();
}

async function newSession() {
    if (currentSessionId !== null) await closeSession(currentSessionId);
    resetDataObject();
    drawAircraftOnMap(data);
    drawAircraftPathOnMap(data);
    drawMissionOnMap(data);
    drawHomeOnMap(data);
    var defaultName = "Flight " + new Date().toISOString().slice(0, 16).replace('T', ' ');
    currentSessionId = await createSession(defaultName);
    await renderSessionsList();
}

async function replaySession(id) {
    bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('bsOffcanvasSessions')).hide();
    var messages = await getSessionMessages(id);
    var lines = messages.map(function(m) { return m.line; });
    replayFromSessionMessages(lines);
}

async function deleteSessionAndRefresh(id) {
    if (id === currentSessionId) {
        alert("Cannot delete the current session.");
        return;
    }
    if (!confirm("Delete this session? This cannot be undone.")) return;
    var overlay = document.getElementById('deletingSessionOverlay');
    overlay.style.display = 'flex';
    try {
        await deleteSession(id);
        await renderSessionsList();
    } finally {
        overlay.style.display = 'none';
    }
}

async function renderSessionsList() {
    var sessions = await listSessions();

    var current = sessions.find(function(s) { return s.id === currentSessionId; });
    if (current) {
        document.getElementById("currentSessionName").value = current.name;
    }

    var container = document.getElementById("sessionsList");
    container.innerHTML = "";

    sessions.forEach(function(session) {
        var isOpen = session.status === 'open';
        var duration = isOpen
            ? Date.now() - session.startTime
            : session.lastUpdate - session.startTime;
        var durationStr = secondsToNiceTime(Math.max(0, Math.floor(duration / 1000)));
        var dateStr = new Date(session.startTime).toLocaleString();

        var item = document.createElement('div');
        item.className = 'session-item';

        var info = document.createElement('div');
        info.className = 'session-info';

        var nameSpan = document.createElement('span');
        nameSpan.className = 'session-name';
        nameSpan.textContent = (isOpen ? '\u25cf ' : '') + session.name;

        var metaSpan = document.createElement('span');
        metaSpan.className = 'session-meta';
        metaSpan.textContent = dateStr + ' \u00b7 ' + durationStr;

        info.appendChild(nameSpan);
        info.appendChild(metaSpan);

        var actions = document.createElement('div');
        actions.className = 'session-actions';

        if (!isOpen) {
            var replayBtn = document.createElement('button');
            replayBtn.className = 'session-btn';
            replayBtn.textContent = 'Replay';
            replayBtn.addEventListener('click', (function(sid) {
                return function() { replaySession(sid); };
            })(session.id));
            actions.appendChild(replayBtn);
        }

        var deleteBtn = document.createElement('button');
        deleteBtn.className = 'session-btn session-btn-delete';
        deleteBtn.textContent = 'Delete';
        deleteBtn.addEventListener('click', (function(sid) {
            return function() { deleteSessionAndRefresh(sid); };
        })(session.id));
        actions.appendChild(deleteBtn);

        item.appendChild(info);
        item.appendChild(actions);
        container.appendChild(item);
    });
}

// ── Monitor other UAVs ────────────────────────────────────────────────────────

function getMonitorTopicPrefix() {
    var primaryTopic = localStorage.getItem("mqttTopic") || "";
    var lastSlash = primaryTopic.lastIndexOf("/");
    return lastSlash >= 0 ? primaryTopic.substring(0, lastSlash + 1) : "";
}

function openMonitorPanel() {
    renderMonitoredList();
    document.getElementById("inputMonitorTopic").value = getMonitorTopicPrefix();
    bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('bsOffcanvasMonitor')).show();
}

function renderMonitoredList() {
    var container = document.getElementById("monitoredList");
    var topics = Object.keys(otherAircraft);
    if (topics.length === 0) {
        container.innerHTML = '<p class="text-secondary small">No aircraft being monitored.</p>';
        return;
    }
    var html = '';
    for (var i = 0; i < topics.length; i++) {
        var entry = otherAircraft[topics[i]];
        var label = entry.callsign ? entry.callsign + ' (' + entry.topic + ')' : entry.topic;
        html += '<div style="display:flex;align-items:center;padding:0.5em 0;border-bottom:1px solid #222;">'
            + '<span style="display:inline-block;width:0.75em;height:0.75em;border-radius:50%;background:' + entry.colour + ';margin-right:0.6em;flex-shrink:0;"></span>'
            + '<span style="flex:1;font-size:0.875em;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;">' + label + '</span>'
            + '<input type="button" class="btRcCmd" value="Remove" data-topic="' + entry.topic + '" style="margin-left:0.5em;flex-shrink:0;font-size:0.8em;padding:0.25em 0.6em;" /></div>';
    }
    container.innerHTML = html;
    var btns = container.querySelectorAll('input[data-topic]');
    for (var j = 0; j < btns.length; j++) {
        btns[j].addEventListener('click', function() {
            var t = this.getAttribute('data-topic');
            removeMonitoredTopic(t);
            removeSecondaryAircraftFromMap(t);
            renderMonitoredList();
        });
    }
}

// ── Secondary aircraft popup ──────────────────────────────────────────────────

function refreshSecondaryPopup() {
    if (!currentPopupTopic) return;
    var entry = otherAircraft[currentPopupTopic];
    if (!entry) return;

    var callsign = entry.callsign || currentPopupTopic.split('/').pop();
    var csEl = document.getElementById("secPopupCallsign");
    csEl.textContent = callsign;
    csEl.style.borderColor = entry.colour;

    var elapsed = entry.lastSeen > 0 ? Math.floor((Date.now() - entry.lastSeen) / 1000) : -1;
    var elapsedStr = elapsed < 0 ? 'Never' : elapsed < 60 ? elapsed + ' s ago' : Math.floor(elapsed / 60) + ' min ago';
    document.getElementById("secPopupLastSeen").innerHTML =
        '<span style="color:#aaa;">Last seen:</span> ' + elapsedStr;

    var olcEl = document.getElementById("secPopupOlc");
    var latDeg = entry.lat / 10000000.0;
    var lonDeg = entry.lon / 10000000.0;
    if (entry.lastSeen > 0 && typeof OpenLocationCode !== 'undefined') {
        var code = OpenLocationCode.encode(latDeg, lonDeg);
        olcEl.textContent = '\uD83D\uDCCD ' + code;
        olcEl.style.display = '';
        olcEl.onclick = function() {
            navigator.clipboard.writeText(code).then(function() {
                document.getElementById("secPopupOlcCopied").style.display = '';
                setTimeout(function() {
                    document.getElementById("secPopupOlcCopied").style.display = 'none';
                }, 2000);
            }).catch(function() { prompt("Copy this Plus Code:", code); });
        };
    } else {
        olcEl.style.display = 'none';
    }

    var altUnit = localStorage.getItem("ui_altitude") || "m";
    var spdUnit = localStorage.getItem("ui_speed")    || "kmh";
    var distUnit = localStorage.getItem("ui_distance") || "km";

    // Altitude ASL
    var aslEl = document.getElementById("secPopupAsl");
    if (entry.asl !== null) {
        aslEl.textContent = altUnit === "ft"
            ? (entry.asl * 3.28084).toFixed(0) + " ft"
            : entry.asl.toFixed(0) + " m";
    } else { aslEl.textContent = "—"; }

    // Vertical speed
    var vspMs = entry.vsp / 100;
    document.getElementById("secPopupVsp").textContent = vspMs.toFixed(1) + " m/s";

    // Horizontal speed
    var gspMs = entry.gsp / 100;
    var gspText;
    switch (spdUnit) {
        case "mph": gspText = (gspMs * 2.23694).toFixed(0) + " mph"; break;
        case "ms":  gspText = gspMs.toFixed(1)              + " m/s"; break;
        case "kt":  gspText = (gspMs * 1.94384).toFixed(0) + " kt";  break;
        default:    gspText = (gspMs * 3.6).toFixed(0)      + " km/h"; break;
    }
    document.getElementById("secPopupGsp").textContent = gspText;

    // Heading
    document.getElementById("secPopupHeading").textContent = entry.heading + "°";

    // Distance and azimuth from primary aircraft
    var distEl = document.getElementById("secPopupDistance");
    var aziEl  = document.getElementById("secPopupAzimuth");
    if (entry.lastSeen > 0 && data.gpsLatitude && data.gpsLongitude) {
        var distM = getDistanceBetweenTwoPoints(data.gpsLatitude, data.gpsLongitude, latDeg, lonDeg);
        var distText;
        if (distUnit === "mi") distText = (distM / 1609.34).toFixed(2) + " mi";
        else if (distUnit === "ft") distText = (distM * 3.28084).toFixed(0) + " ft";
        else distText = distM >= 1000 ? (distM / 1000).toFixed(2) + " km" : distM.toFixed(0) + " m";
        distEl.textContent = distText;

        var dLat = (latDeg - data.gpsLatitude) * Math.PI / 180;
        var dLon = (lonDeg - data.gpsLongitude) * Math.PI / 180;
        var lat1r = data.gpsLatitude * Math.PI / 180;
        var lat2r = latDeg * Math.PI / 180;
        var sinX = Math.sin(dLon) * Math.cos(lat2r);
        var cosX = Math.cos(lat1r) * Math.sin(lat2r) - Math.sin(lat1r) * Math.cos(lat2r) * Math.cos(dLon);
        var brng = (Math.atan2(sinX, cosX) * 180 / Math.PI + 360) % 360;
        aziEl.textContent = brng.toFixed(0) + "°";
    } else {
        distEl.textContent = "—";
        aziEl.textContent  = "—";
    }

    // Battery
    var batEl = document.getElementById("secPopupBattery");
    batEl.textContent = entry.batteryPercent !== null ? entry.batteryPercent + "%" : "—";

    // Flight time
    var ftEl = document.getElementById("secPopupFlightTime");
    ftEl.textContent = entry.flightTime !== null ? secondsToNiceTime(entry.flightTime) : "—";

    // Home distance
    var hdEl = document.getElementById("secPopupHomeDist");
    if (entry.homeDistance !== null) {
        var hdM = entry.homeDistance;
        var hdText;
        if (distUnit === "mi") hdText = (hdM / 1609.34).toFixed(2) + " mi";
        else hdText = hdM >= 1000 ? (hdM / 1000).toFixed(2) + " km" : hdM.toFixed(0) + " m";
        hdEl.textContent = hdText;
    } else { hdEl.textContent = "—"; }
}

function showSecondaryAircraftPopup(topic) {
    var entry = otherAircraft[topic];
    if (!entry) return;
    currentPopupTopic = topic;
    document.getElementById("secPopupOlcCopied").style.display = 'none';
    refreshSecondaryPopup();
    document.getElementById("secAircraftPopup").style.display = 'block';
    document.getElementById("secAircraftPopupOverlay").style.display = 'block';
}

function closeSecondaryPopup() {
    document.getElementById("secAircraftPopup").style.display = 'none';
    document.getElementById("secAircraftPopupOverlay").style.display = 'none';
    currentPopupTopic = null;
}

// ── Misc helpers ──────────────────────────────────────────────────────────────

function isRunningStandalone() {
    return (window.navigator.standalone === true) ||
           (window.matchMedia('(display-mode: standalone)').matches);
}

// ── Resize handler ────────────────────────────────────────────────────────────

window.onresize = function() {
    renderEFIS(data);
};

// ── Event listener wiring ────────────────────────────────────────────────────

function wireEventListeners() {
    // Main sidebar nav
    document.getElementById("navSendCommand").addEventListener("click", function(e) {
        e.preventDefault();
        bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('sideMenu')).hide();
        openCommandsModal();
    });

    document.getElementById("navFlightSessions").addEventListener("click", function(e) {
        e.preventDefault();
        bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('sideMenu')).hide();
        openSessionsPanel();
    });

    document.getElementById("navMonitorUAVs").addEventListener("click", function(e) {
        e.preventDefault();
        bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('sideMenu')).hide();
        openMonitorPanel();
    });

    document.getElementById("navMissionPlanner").addEventListener("click", function(e) {
        e.preventDefault();
        bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('sideMenu')).hide();
        openMissionPlanner();
    });

    document.getElementById("navSettings").addEventListener("click", function(e) {
        e.preventDefault();
        bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('sideMenu')).hide();
        document.getElementById("installhomelink").style.display = isRunningStandalone() ? "none" : "";
        bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('bsOffcanvasSettings')).show();
    });

    document.getElementById("navRefreshApp").addEventListener("click", function(e) {
        e.preventDefault();
        window.location.reload();
    });

    // Settings offcanvas nav
    document.getElementById("navBrokerSettings").addEventListener("click", function(e) {
        e.preventDefault();
        openBrokerSettings();
    });

    document.getElementById("uisettingslink").addEventListener("click", function(e) {
        e.preventDefault();
        openUISettings();
    });

    document.getElementById("navSecurityMenu").addEventListener("click", function(e) {
        e.preventDefault();
        openSecurityModal();
    });

    document.getElementById("navKeepAwake").addEventListener("click", function(e) {
        e.preventDefault();
        bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('bsOffcanvasSettings')).hide();
        noSleep.enable();
    });

    document.getElementById("navEnableCompass").addEventListener("click", function(e) {
        e.preventDefault();
        startOrientationTracking().then(function(granted) {
            if (granted) {
                document.getElementById("navEnableCompass").textContent = "Compass enabled \u2713";
            } else {
                alert("Compass permission was denied.");
            }
        });
        bootstrap.Offcanvas.getOrCreateInstance(document.getElementById('bsOffcanvasSettings')).hide();
    });

    // Broker settings modal
    document.getElementById("btSaveBrokerSettings").addEventListener("click",   saveBrokerSettings);
    document.getElementById("btResetBrokerSettings").addEventListener("click",  resetBrokerSettings);

    // UI settings modal
    document.getElementById("btSaveUISettings").addEventListener("click",       saveUISettings);

    // Security modal
    document.getElementById("btGenerateKey").addEventListener("click",          generateKeyPair);
    document.getElementById("btCopyPublicKey").addEventListener("click",        copyPublicKey);

    // Sessions offcanvas
    document.getElementById("btRenameSession").addEventListener("click",        renameCurrentSession);
    document.getElementById("btNewSession").addEventListener("click",           newSession);
    document.getElementById("navExportSession").addEventListener("click", function() {
        savemqttlog();
    });
    document.getElementById("navImportSession").addEventListener("click",       replaymqttlog);
    document.getElementById("btStopReplay").addEventListener("click",           stopreplaymqttlog);

    // Monitor UAVs offcanvas
    document.getElementById("btAddMonitorTopic").addEventListener("click", function() {
        var topicInput = document.getElementById("inputMonitorTopic");
        var errorEl    = document.getElementById("monitorTopicError");
        var err = addMonitoredTopic(topicInput.value.trim());
        if (err) {
            errorEl.textContent = err;
            errorEl.style.display = '';
        } else {
            errorEl.style.display = 'none';
            topicInput.value = getMonitorTopicPrefix();
            renderMonitoredList();
        }
    });

    // FAB button
    document.getElementById("fabCommands").addEventListener("click", openCommandsModal);

    // Commands modal buttons
    document.getElementById("btSendPing").addEventListener("click", function() {
        if (!mqttConnected || data.downlinkStatus !== 1) return;
        publishCommand("ping");
        updateCommandsPanel();
    });

    document.getElementById("btSendHeading").addEventListener("click", function() {
        if (!mqttConnected || data.downlinkStatus !== 1) return;
        var input = document.getElementById("inputHeading");
        var val = parseInt(input.value, 10);
        if (isNaN(val) || val < 0 || val > 359) {
            alert("Please enter a heading between 0 and 359 degrees.");
            input.value = "";
            return;
        }
        publishCommand("setheading", null, { heading: val }, "setheading:" + val + "\u00b0");
        updateCommandsPanel();
    });

    document.getElementById("btSendJumpWp").addEventListener("click", function() {
        if (!mqttConnected || data.downlinkStatus !== 1) return;
        var input = document.getElementById("inputWpIndex");
        var val = parseInt(input.value, 10);
        var maxWp = data.waypointCount > 0 ? data.waypointCount : 255;
        if (isNaN(val) || val < 1 || val > maxWp) {
            alert("Please enter a waypoint number between 1 and " + maxWp + ".");
            input.value = "";
            return;
        }
        publishCommand("jumpwp", null, { wp: val - 1 }, "jumpwp:" + val);
        updateCommandsPanel();
    });

    document.getElementById("btSendAltitude").addEventListener("click", function() {
        if (!mqttConnected || data.downlinkStatus !== 1) return;
        var altUnit = localStorage.getItem("ui_altitude") || "m";
        var input = document.getElementById("inputAltitude");
        var rawVal = parseFloat(input.value);
        if (isNaN(rawVal) || rawVal < -10000 || rawVal > 100000) {
            alert("Please enter an altitude between -10000 and 100000 " + (altUnit === "ft" ? "ft" : "m") + ".");
            input.value = "";
            return;
        }
        var altM = (altUnit === "ft") ? rawVal * 0.3048 : rawVal;
        var altCm = Math.round(altM * 100);
        var label = "setalt:" + rawVal + (altUnit === "ft" ? "ft" : "m");
        publishCommand("setalt", null, { alt: altCm }, label);
        updateCommandsPanel();
    });

    (function() {
        for (var i = 0; i < rcCommands.length; i++) {
            (function(entry) {
                document.getElementById(entry.onId).addEventListener("click", function() {
                    if (!mqttConnected || data.downlinkStatus !== 1 || data.mspRcOverride !== 1) return;
                    publishCommand(entry.cmd, 1);
                });
                document.getElementById(entry.offId).addEventListener("click", function() {
                    if (!mqttConnected || data.downlinkStatus !== 1 || data.mspRcOverride !== 1) return;
                    publishCommand(entry.cmd, 0);
                });
            })(rcCommands[i]);
        }
    })();

    // Secondary aircraft popup
    document.getElementById("btSecPopupStop").addEventListener("click", function() {
        if (!currentPopupTopic) return;
        var cs = otherAircraft[currentPopupTopic] ? otherAircraft[currentPopupTopic].callsign || currentPopupTopic : currentPopupTopic;
        if (!confirm('Stop tracking ' + cs + '?')) return;
        removeMonitoredTopic(currentPopupTopic);
        removeSecondaryAircraftFromMap(currentPopupTopic);
        renderMonitoredList();
        closeSecondaryPopup();
    });
    document.getElementById("btSecPopupClose").addEventListener("click",         closeSecondaryPopup);
    document.getElementById("secAircraftPopupOverlay").addEventListener("click", closeSecondaryPopup);
}

// ── Update notification ───────────────────────────────────────────────────────

function checkforNewUIVersion() {
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        if (xmlhttp.readyState !== XMLHttpRequest.DONE) return;
        if (xmlhttp.status === 200) {
            try {
                var json = JSON.parse(xmlhttp.responseText);
                if (json.lastVersion !== currentVersion) {
                    console.log("New UI version available!");
                    document.getElementById("refreshMenuBadge").style.display = "inline";
                    document.getElementById("gearIconBadge").style.display    = "block";
                }
            } catch(e) { /* ignore parse errors */ }
        }
    };
    xmlhttp.open("GET", "uiversion.json?t=" + Date.now(), true);
    xmlhttp.send();
}

// ── DOMContentLoaded ─────────────────────────────────────────────────────────

window.addEventListener("DOMContentLoaded", async function() {

    // ── Session management ─────────────────────────────────────────────────
    try {
        await openDB();
        var openSession = await getOpenSession();
        if (openSession) {
            var messageCount = await countSessionMessages(openSession.id);
            var loadExisting = true;
            if (messageCount > 10000) {
                loadExisting = window.confirm(
                    "Your current session \"" + openSession.name + "\" has " +
                    messageCount.toLocaleString() + " messages and may take a long time to load.\n\n" +
                    "Press OK to load it anyway, or Cancel to close it and start a fresh session."
                );
                if (!loadExisting) await closeSession(openSession.id);
            }
            if (loadExisting) {
                currentSessionId = openSession.id;
                var storedMessages = await getSessionMessages(openSession.id);
                restoreFromSessionMessages(storedMessages.map(function(m) { return m.line; }));
            } else {
                var defaultName = "Flight " + new Date().toISOString().slice(0, 16).replace('T', ' ');
                currentSessionId = await createSession(defaultName);
            }
        } else {
            var defaultName = "Flight " + new Date().toISOString().slice(0, 16).replace('T', ' ');
            currentSessionId = await createSession(defaultName);
        }
    } catch (e) {
        console.error("Session storage init failed:", e);
    }

    setOnMessageCallback(function(line) {
        if (currentSessionId !== null) appendMessage(currentSessionId, line);
    });

    setOnReplayStop(async function() {
        if (currentSessionId !== null) {
            try {
                var msgs = await getSessionMessages(currentSessionId);
                restoreFromSessionMessages(msgs.map(function(m) { return m.line; }));
            } catch (e) {
                console.error("Session restore after replay failed:", e);
            }
        }
    });

    // ── Waypoint click → Jump to WP ───────────────────────────────────────
    setOnWaypointClick(function(wpNumber) {
        if (data.fmWp !== 1) return;
        if (!mqttConnected || data.downlinkStatus !== 1) return;
        if (data.extCmdsSupported < 1) return;
        if (confirm("Jump to WP " + wpNumber + "?")) {
            publishCommand("jumpwp", null, { wp: wpNumber - 1 }, "jumpwp:" + wpNumber);
        }
    });

    setOnSecondaryAircraftClick(showSecondaryAircraftPopup);

    // ── Wire all UI events ────────────────────────────────────────────────
    wireEventListeners();

    // ── Mission Planner ───────────────────────────────────────────────────
    initMissionPlannerButtons();

    // ── iOS compass permission button ─────────────────────────────────────
    if (typeof DeviceOrientationEvent !== 'undefined' &&
        typeof DeviceOrientationEvent.requestPermission === 'function') {
        document.getElementById('navEnableCompass').style.display = '';
    } else {
        startOrientationTracking();
    }

    // ── Connect and initial render ────────────────────────────────────────
    MQTTconnect();
    loadAndSubscribeMonitoredTopics();
    setUIUnits();

    renderEFIS(data);
    updateDataView(data);
    drawAircraftOnMap(data);
    getUserLocation();
    drawUserOnMap(data);

    // ── Timer loop ────────────────────────────────────────────────────────

    setInterval(function() {
        estimateEfis();
        renderEFIS(data);
    }, pageSettings.efisRefreshInterval);

    setInterval(function() {
        estimatePosition();
        drawAircraftOnMap(data);
        drawCourseLineOnMap(data);
        updateSecondaryAircraftOnMap();
        refreshSecondaryPopup();
    }, pageSettings.mapFastRefreshInterval);

    setInterval(function() {
        toggleBlinkFast();
        drawAircraftPathOnMap(data);
        drawUserOnMap(data);
        updateDataView(data);

        if (document.getElementById('bsModalCommands').classList.contains('show'))
            updateCommandsPanel();
        if (document.getElementById('bsModalSecurity').classList.contains('show'))
            loadSecurityPanel();
    }, pageSettings.mapAndDataRefreshInterval);

    setInterval(function() {
        data.powerTime++;
        if (data.uavIsArmed) {
            data.flightTime++;
            if (data.currentFlightWaypoints.length > 3600)
                data.currentFlightWaypoints.shift();
            var n = data.currentFlightWaypoints.length;
            data.currentFlightWaypoints[n] = {
                wpLatitude:  data.gpsLatitude,
                wpLongitude: data.gpsLongitude,
            };
        }
        for (var topic in otherAircraft) {
            var sec = otherAircraft[topic];
            if (sec.flightTime !== null) sec.flightTime++;
        }
    }, 1000);

    var lastVersionCheckTime = Date.now();
    checkforNewUIVersion();

    setInterval(function() {
        drawMissionOnMap(data);
        drawHomeOnMap(data);
        if (user_moved_map) setUserMovedMap(false);
        else centerMap(data);
        if (data.waypointCount > 0 && data.isCurrentMissionElevationSet === false &&
            data.isWaypointMissionValid === 1 && updatingWpAltitudes === false &&
            data.waypointCount === (data.currentMissionWaypoints.length - 1)) {
            getMissionWaypointsAltitude();
        }
        if (Date.now() - lastVersionCheckTime > 15000) {
            checkforNewUIVersion();
            lastVersionCheckTime = Date.now();
        }
    }, pageSettings.lowPriorityTasksInterval);

    setInterval(function() {
        toggleBlinkSlow();
    }, 2000);

});
