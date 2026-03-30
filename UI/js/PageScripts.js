import { data, mqtt, mqttConnected, MQTTconnect, MQTTSetDefaultSettings, savemqttlog, replaymqttlog, stopreplaymqttlog, resetDataObject, pageSettings, estimateEfis, estimatePosition, updatingWpAltitudes, setOnMessageCallback, setOnReplayStop, replayFromSessionMessages, restoreFromSessionMessages, secondsToNiceTime, publishCommand, commandHistory, otherAircraft, addMonitoredTopic, removeMonitoredTopic, loadAndSubscribeMonitoredTopics } from './CommScripts.js';
import { openDB, createSession, closeSession, getOpenSession, listSessions, getSessionMessages, countSessionMessages, appendMessage, deleteSession, renameSession } from './SessionScripts.js';
import { efis, renderEFIS } from './EfisScripts.js';
import { drawAircraftOnMap, drawAircraftPathOnMap, drawCourseLineOnMap, drawMissionOnMap, drawHomeOnMap, drawUserOnMap, centerMap, getMissionWaypointsAltitude, getUserLocation, startOrientationTracking, user_moved_map, setUserMovedMap, setMapStyle, setOnWaypointClick, updateSecondaryAircraftOnMap, removeSecondaryAircraftFromMap, setOnSecondaryAircraftClick } from './MapScripts.js';
import { updateDataView, setUIUnits, toggleBlinkFast, toggleBlinkSlow, openGoogleMaps } from './InfoPanelScripts.js';

// Setup viewport
var windowOuterWidth = 0;

function UpdateViewPortSize(resize=true) {
    if(windowOuterWidth != window.outerWidth) {
        windowOuterWidth = window.outerWidth;
        var width = windowOuterWidth * window.devicePixelRatio;
        var viewport = document.querySelector("meta[name=viewport]");
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
        localStorage.setItem("mqttTopic", "bulletgcss/telem/your_callsign");
    if(localStorage.getItem("mqttCommandTopic") === null)
        localStorage.setItem("mqttCommandTopic", "bulletgcss/cmd/your_callsign");
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

    if(localStorage.getItem("ui_map_style") === null)
        localStorage.setItem("ui_map_style", "liberty");

    if(localStorage.getItem("ui_elevation_provider") === null)
        if(location.href.indexOf("bulletgcss.outros.net") > 0)
            localStorage.setItem("ui_elevation_provider", "OpenTopoDataDirect" );
        else
            localStorage.setItem("ui_elevation_provider", "OpenTopoData" );
    }

checkForDefaultSettings();

// ─── Session management ────────────────────────────────────────────────────────

var currentSessionId = null;

function openCommandsMenu() {
    updateCommandsPanel();
    document.getElementById("commandsMenu").style.width = "100%";
    closeNav();
}

function closeCommandsMenu() {
    document.getElementById("commandsMenu").style.width = "0";
}

// RC channel commands table — maps button IDs to firmware telemetry state fields.
// dataKey: the key in the `data` object that the firmware reports (null = not yet received).
var rcCommands = [
    { cmd: "rth",     onId: "btRthOn",     offId: "btRthOff",     dataKey: "cmdRth"     },
    { cmd: "althold", onId: "btAltHoldOn", offId: "btAltHoldOff", dataKey: "cmdAltHold" },
    { cmd: "cruise",  onId: "btCruiseOn",  offId: "btCruiseOff",  dataKey: "cmdCruise"  },
    { cmd: "beeper",  onId: "btBeeperOn",  offId: "btBeeperOff",  dataKey: "cmdBeeper"  },
    { cmd: "wp",      onId: "btWpOn",      offId: "btWpOff",      dataKey: "cmdWp",      onCondition: function() { return data.isWaypointMissionValid === 1; } },
    { cmd: "poshold", onId: "btPosHoldOn", offId: "btPosHoldOff", dataKey: "cmdPosHold" },
];

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

    // Keep Jump to WP input bounded by the loaded mission's waypoint count
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
        offBtn.disabled = !modeActive;  // only enabled when firmware confirms mode is active

        onBtn.classList.toggle('btn-active', modeActive);
        onBtn.classList.remove('btn-inactive');
        offBtn.classList.remove('btn-active', 'btn-inactive');
    }

    renderCommandHistory();
}

function openSecurityMenu() {
    loadSecurityPanel();
    document.getElementById("securityMenu").style.width = "100%";
    closeNav();
}

function closeSecurityMenu() {
    document.getElementById("securityMenu").style.width = "0";
}

// ─── Multi-aircraft monitoring panel ─────────────────────────────────────────

function openMonitoredMenu() {
    renderMonitoredList();
    document.getElementById("monitoredUAVsMenu").style.width = "100%";
    closeNav();
}

function closeMonitoredMenu() {
    document.getElementById("monitoredUAVsMenu").style.width = "0";
}

function renderMonitoredList() {
    var container = document.getElementById("monitoredList");
    var topics = Object.keys(otherAircraft);
    if (topics.length === 0) {
        container.innerHTML = '<p style="color:#aaa;margin:2vmin 5vmin;font-size:3.5vmin;">No aircraft being monitored.</p>';
        return;
    }
    var html = '';
    for (var i = 0; i < topics.length; i++) {
        var entry = otherAircraft[topics[i]];
        var label = entry.callsign ? entry.callsign + ' (' + entry.topic + ')' : entry.topic;
        html += '<div style="display:flex;align-items:center;padding:2vmin 5vmin;border-bottom:1px solid #333;">' +
            '<span style="display:inline-block;width:3vmin;height:3vmin;border-radius:50%;background:' + entry.colour + ';margin-right:2vmin;flex-shrink:0;"></span>' +
            '<span style="flex:1;font-size:3.5vmin;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;">' + label + '</span>' +
            '<input type="button" class="btRcCmd" value="Remove" data-topic="' + entry.topic + '" style="margin-left:2vmin;flex-shrink:0;" /></div>';
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

// ─── Secondary aircraft popup ─────────────────────────────────────────────────

var currentPopupTopic = null;

function refreshSecondaryPopup() {
    if (!currentPopupTopic) return;
    var entry = otherAircraft[currentPopupTopic];
    if (!entry) return;

    var callsign = entry.callsign || currentPopupTopic.split('/').pop();
    var csEl = document.getElementById("secPopupCallsign");
    csEl.textContent = callsign;
    csEl.style.borderColor = entry.colour;

    var elapsed = entry.lastSeen > 0 ? Math.floor((Date.now() - entry.lastSeen) / 1000) : -1;
    document.getElementById("secPopupLastSeen").textContent = 'Last seen: ' +
        (elapsed < 0 ? 'Never' : elapsed < 60 ? elapsed + ' s ago' : Math.floor(elapsed / 60) + ' min ago');

    var olcEl = document.getElementById("secPopupOlc");
    if (entry.lastSeen > 0 && typeof OpenLocationCode !== 'undefined') {
        var latDeg = entry.lat / 10000000.0;
        var lonDeg = entry.lon / 10000000.0;
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

// ──────────────────────────────────────────────────────────────────────────────

function loadSecurityPanel() {
    var hasKey     = localStorage.getItem("commandPrivateKey") !== null;
    var uiBase64   = localStorage.getItem("commandPublicKeyBase64");

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

    var fwBase64   = data.firmwarePublicKey; // "" = not yet received from firmware
    var allZeros   = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="; // base64 of 32 zero bytes

    var statusEl   = document.getElementById("securityKeyStatus");
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
    document.getElementById("btCopyPublicKey").style.display = hasKey ? "block" : "none";
    document.getElementById("securityKeyHint").style.display = hasKey ? "block" : "none";
    document.getElementById("btGenerateKey").value = hasKey ? "Regenerate key pair" : "Generate key pair";
}

async function generateKeyPair() {
    var hasKey = localStorage.getItem("commandPrivateKey") !== null;
    if (hasKey) {
        if (!confirm("Generating a new key pair will require re-flashing the firmware with the new public key. The existing key pair will be lost.\n\nContinue?")) {
            return;
        }
    }
    try {
        var keyPair = await window.crypto.subtle.generateKey(
            { name: "Ed25519" },
            true,
            ["sign", "verify"]
        );
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

function renderCommandHistory() {
    var container = document.getElementById("commandsList");
    if (commandHistory.length === 0) {
        container.innerHTML = '<p style="color:#aaa; margin-left:5vmin;">No commands sent yet.</p>';
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

function openSessionsMenu() {
    renderSessionsList().then(function() {
        document.getElementById("sessionsMenu").style.width = "100%";
        closeNav();
    });
}

function closeSessionsMenu() {
    document.getElementById("sessionsMenu").style.width = "0";
}

async function renameCurrentSession() {
    if (currentSessionId === null) return;
    var newName = document.getElementById("currentSessionName").value.trim();
    if (!newName) return;
    await renameSession(currentSessionId, newName);
    await renderSessionsList();
}

async function newSession() {
    if (currentSessionId !== null) {
        await closeSession(currentSessionId);
    }
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
    closeSessionsMenu();
    var messages = await getSessionMessages(id);
    var lines = messages.map(function(m) { return m.line; });
    replayFromSessionMessages(lines);
    // Playback controls (Stop Replay, slider) are now inside sessionsMenu.
    // User can reopen Sessions to control playback.
}

async function deleteSessionAndRefresh(id) {
    if (id === currentSessionId) {
        alert("Cannot delete the current session.");
        return;
    }
    if (!confirm("Delete this session? This cannot be undone.")) return;
    await deleteSession(id);
    await renderSessionsList();
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

// ──────────────────────────────────────────────────────────────────────────────

// Setup Sidebar
function openNav() {
    // If App is running standalone, then don't show the option to install
    document.getElementById("installhomelink").style.display = isRunningStandalone() ? "none" : "";
    // Open menu
    document.getElementById("sideMenu").style.width = "100%";
}

function openSettingsNav() {
    // Not implemented yet
    document.getElementById("missionplannerlink").style.display = "none";
    document.getElementById("inavsettingslink").style.display = "none";
    openSettingsMenu();
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
    document.getElementById("brokerCommandTopic").value = localStorage.getItem("mqttCommandTopic");
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
    localStorage.setItem("mqttCommandTopic", document.getElementById("brokerCommandTopic").value);
    localStorage.setItem("mqttUseTLS", document.getElementById("brokerUseTLS").checked ? "true" : "false");

    if(mqttConnected)
        mqtt.disconnect();

    closeBrokerSettings();
    closeNav();
}

function resetBrokerSettings()
{
    MQTTSetDefaultSettings();

    if(mqttConnected)
        mqtt.disconnect();

    closeBrokerSettings();
    closeNav();
}

function closeBrokerSettings() {
    document.getElementById("brokerSettings").style.width = "0";
}

function openSettingsMenu() {
    document.getElementById("settingsMenu").style.width = "100%";
    closeNav();
}

function closeSettingsMenu() {
    document.getElementById("settingsMenu").style.width = "0";
}

function openUISettings()
{
    document.getElementById("ui_speed").value = localStorage.getItem("ui_speed");
    document.getElementById("ui_distance").value = localStorage.getItem("ui_distance");
    document.getElementById("ui_altitude").value = localStorage.getItem("ui_altitude");
    document.getElementById("ui_current").value = localStorage.getItem("ui_current");
    document.getElementById("ui_capacity").value = localStorage.getItem("ui_capacity");
    document.getElementById("ui_efficiency").value = localStorage.getItem("ui_efficiency");
    document.getElementById("ui_map_style").value = localStorage.getItem("ui_map_style") || "liberty";
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
    setMapStyle(document.getElementById("ui_map_style").value);
    localStorage.setItem("ui_elevation_provider", document.getElementById("ui_elevation_provider").value );

    setUIUnits();
    closeUISettings();
    closeNav();
}

function closeUISettings() {
    document.getElementById("uiSettings").style.width = "0";
}

function getRadioValue(radioName) {
    var ret = "";
    var radios = document.getElementsByName(radioName);
    for (var i=0; i < radios.length; i++)
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
    for (var i=0; i < radios.length; i++)
    {
        if(radios[i].value == value)
        {
            radios[i].checked = true;
            break;
        }
    }
}

function reloadApplication()
{
    closeNav();
    window.location.reload();
}

function isRunningStandalone()
{
    var isInWebAppiOS = (window.navigator.standalone === true);
    var isInWebAppChrome = (window.matchMedia('(display-mode: standalone)').matches);

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

window.onresize = function() {
    renderEFIS(data);
};

setUIUnits();

// Wire up event listeners for all interactive elements (replaces inline onclick)
document.getElementById("gearIcon").addEventListener("click", openNav);
document.getElementById("closeSideMenu").addEventListener("click", closeNav);
document.getElementById("navSettingsMenu").addEventListener("click", openSettingsNav);
document.getElementById("closeSettingsMenu").addEventListener("click", closeSettingsMenu);
document.getElementById("navBrokerSettings").addEventListener("click", openBrokerSettings);
document.getElementById("uisettingslink").addEventListener("click", openUISettings);
document.getElementById("navKeepAwake").addEventListener("click", keepScreenAwake);
document.getElementById("navEnableCompass").addEventListener("click", function() {
    startOrientationTracking().then(function(granted) {
        if (granted) {
            document.getElementById("navEnableCompass").textContent = "Compass enabled \u2713";
        } else {
            alert("Compass permission was denied.");
        }
    });
    closeNav();
});
document.getElementById("navRefreshApp").addEventListener("click", reloadApplication);
document.getElementById("btSaveBrokerSettings").addEventListener("click", saveBrokerSettings);
document.getElementById("btResetBrokerSettings").addEventListener("click", resetBrokerSettings);
document.getElementById("closeBrokerSettings").addEventListener("click", closeBrokerSettings);
document.getElementById("closeUISettings").addEventListener("click", closeUISettings);
document.getElementById("btSaveUISettings").addEventListener("click", saveUISettings);
document.getElementById("navExportSession").addEventListener("click", function() { savemqttlog(); closeNav(); });
document.getElementById("navImportSession").addEventListener("click", replaymqttlog);
document.getElementById("btStopReplay").addEventListener("click", stopreplaymqttlog);
document.getElementById("senduavcommandlink").addEventListener("click", openCommandsMenu);
document.getElementById("fabCommands").addEventListener("click", openCommandsMenu);
document.getElementById("navSecurityMenu").addEventListener("click", openSecurityMenu);
document.getElementById("closeSecurityMenu").addEventListener("click", closeSecurityMenu);
document.getElementById("btGenerateKey").addEventListener("click", generateKeyPair);
document.getElementById("btCopyPublicKey").addEventListener("click", copyPublicKey);
document.getElementById("closeCommandsMenu").addEventListener("click", closeCommandsMenu);
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
    // INAV Configurator displays WPs as 1-based; firmware expects 0-based index
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
    // Convert to centimetres for INAV (MSP2_INAV_SET_ALT_TARGET expects cm relative to home)
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
document.getElementById("navSessionsMenu").addEventListener("click", openSessionsMenu);
document.getElementById("closeSessionsMenu").addEventListener("click", closeSessionsMenu);
document.getElementById("btRenameSession").addEventListener("click", renameCurrentSession);
document.getElementById("btNewSession").addEventListener("click", newSession);
document.getElementById("navMonitoredUAVs").addEventListener("click", openMonitoredMenu);
document.getElementById("closeMonitoredMenu").addEventListener("click", closeMonitoredMenu);
document.getElementById("btAddMonitorTopic").addEventListener("click", function() {
    var topicInput = document.getElementById("inputMonitorTopic");
    var errorEl    = document.getElementById("monitorTopicError");
    var err = addMonitoredTopic(topicInput.value.trim());
    if (err) {
        errorEl.textContent = err;
        errorEl.style.display = '';
    } else {
        errorEl.style.display = 'none';
        topicInput.value = '';
        renderMonitoredList();
    }
});
document.getElementById("btSecPopupStop").addEventListener("click", function() {
    if (currentPopupTopic) {
        removeMonitoredTopic(currentPopupTopic);
        removeSecondaryAircraftFromMap(currentPopupTopic);
        renderMonitoredList();
    }
    closeSecondaryPopup();
});
document.getElementById("btSecPopupClose").addEventListener("click", closeSecondaryPopup);
document.getElementById("secAircraftPopupOverlay").addEventListener("click", closeSecondaryPopup);

// Modules are deferred — use DOMContentLoaded instead of window.onload
// to guarantee the handler runs even if the page loaded before the module executed.
window.addEventListener("DOMContentLoaded", async function() {
    // Initialize flight session storage
    try {
        await openDB();
        var openSession = await getOpenSession();
        if (openSession) {
            var messageCount = await countSessionMessages(openSession.id);
            var loadExisting = true;
            if (messageCount > 10000) {
                loadExisting = window.confirm(
                    "Your current session \"" + openSession.name + "\" has " + messageCount.toLocaleString() + " messages and may take a long time to load.\n\n" +
                    "Press OK to load it anyway, or Cancel to close it and start a fresh session."
                );
                if (!loadExisting) {
                    await closeSession(openSession.id);
                }
            }
            if (loadExisting) {
                currentSessionId = openSession.id;
                var storedMessages = await getSessionMessages(openSession.id);
                var storedLines = storedMessages.map(function(m) { return m.line; });
                restoreFromSessionMessages(storedLines);
                console.log("Session restored: " + openSession.name + " (" + storedLines.length + " messages)");
            } else {
                var defaultName = "Flight " + new Date().toISOString().slice(0, 16).replace('T', ' ');
                currentSessionId = await createSession(defaultName);
                console.log("New session created: " + defaultName);
            }
        } else {
            var defaultName = "Flight " + new Date().toISOString().slice(0, 16).replace('T', ' ');
            currentSessionId = await createSession(defaultName);
            console.log("New session created: " + defaultName);
        }
    } catch (e) {
        console.error("Session storage init failed:", e);
    }

    // Record every live MQTT message into the active session
    setOnMessageCallback(function(line) {
        if (currentSessionId !== null) {
            appendMessage(currentSessionId, line);
        }
    });

    // After a session replay ends, restore the live session state
    setOnReplayStop(async function() {
        if (currentSessionId !== null) {
            try {
                var msgs = await getSessionMessages(currentSessionId);
                var lns = msgs.map(function(m) { return m.line; });
                restoreFromSessionMessages(lns);
            } catch (e) {
                console.error("Session restore after replay failed:", e);
            }
        }
    });

    setOnWaypointClick(function(wpNumber) {
        if (data.fmWp !== 1) return;
        if (!mqttConnected || data.downlinkStatus !== 1) return;
        if (data.extCmdsSupported < 1) return;
        if (confirm("Jump to WP " + wpNumber + "?")) {
            publishCommand("jumpwp", null, { wp: wpNumber - 1 }, "jumpwp:" + wpNumber);
        }
    });

    setOnSecondaryAircraftClick(showSecondaryAircraftPopup);

    MQTTconnect();
    loadAndSubscribeMonitoredTopics();
    renderEFIS(data);
    updateDataView(data);
    drawAircraftOnMap(data);

    getUserLocation();
    drawUserOnMap(data);

    // Compass heading via DeviceOrientationEvent.
    // iOS 13+ requires a permission prompt triggered by a user gesture — show the
    // sidebar button and let the user tap it. On all other platforms start immediately.
    if (typeof DeviceOrientationEvent !== 'undefined' &&
        typeof DeviceOrientationEvent.requestPermission === 'function') {
        document.getElementById('navEnableCompass').style.display = '';
    } else {
        startOrientationTracking();
    }

    var timerEFIS = setInterval(function(){
        estimateEfis();
        renderEFIS(data);
    }, pageSettings.efisRefreshInterval);

    var timerMapFast = setInterval(function(){
        estimatePosition();
        drawAircraftOnMap(data);
        drawCourseLineOnMap(data);
        updateSecondaryAircraftOnMap();
        refreshSecondaryPopup();
    }, pageSettings.mapFastRefreshInterval);

    var timerMapAndData = setInterval(function(){
        toggleBlinkFast();

        drawAircraftPathOnMap(data);
        drawUserOnMap(data);
        updateDataView(data);

        if (document.getElementById("commandsMenu").style.width === "100%")
            updateCommandsPanel();
        if (document.getElementById("securityMenu").style.width === "100%")
            loadSecurityPanel();
    }, pageSettings.mapAndDataRefreshInterval);

    var timerOneSecond = setInterval(function(){
        data.powerTime++;
        if(data.uavIsArmed)
        {
            data.flightTime++;

            if(data.currentFlightWaypoints.length > 3600)
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

        if(user_moved_map == true)
            setUserMovedMap(false);
        else
            centerMap(data);

        if(data.waypointCount > 0 && data.isCurrentMissionElevationSet == false
        && data.isWaypointMissionValid == 1 && updatingWpAltitudes == false
        && data.waypointCount == (data.currentMissionWaypoints.length - 1))
        {
            getMissionWaypointsAltitude();
        }

        var uiUpdateNow = new Date().getTime();
        if(uiUpdateNow - lastTimeUIwasOpen > 15000)
        {
            checkforNewUIVersion();
        }
        lastTimeUIwasOpen = uiUpdateNow;

    }, pageSettings.lowPriorityTasksInterval);

    var timerBlinkSlow = setInterval(function(){
        toggleBlinkSlow();
    }, 2000);

});
