import { data, mqtt, mqttConnected, MQTTconnect, MQTTSetDefaultSettings, savemqttlog, replaymqttlog, stopreplaymqttlog, resetDataObject, pageSettings, estimateEfis, estimatePosition, updatingWpAltitudes, setOnMessageCallback, setOnReplayStop, replayFromSessionMessages, restoreFromSessionMessages, secondsToNiceTime } from './CommScripts.js';
import { openDB, createSession, closeSession, getOpenSession, listSessions, getSessionMessages, appendMessage, deleteSession, renameSession } from './SessionScripts.js';
import { efis, renderEFIS } from './EfisScripts.js';
import { drawAircraftOnMap, drawAircraftPathOnMap, drawCourseLineOnMap, drawMissionOnMap, drawHomeOnMap, drawUserOnMap, centerMap, getMissionWaypointsAltitude, getUserLocation, user_moved_map, setUserMovedMap } from './MapScripts.js';
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
        if(location.href.indexOf("bulletgcss.outros.net") > 0)
            localStorage.setItem("ui_elevation_provider", "OpenTopoDataDirect" );
        else
            localStorage.setItem("ui_elevation_provider", "OpenTopoData" );
    }

checkForDefaultSettings();

// ─── Session management ────────────────────────────────────────────────────────

var currentSessionId = null;

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
    var defaultName = "Flight " + new Date().toISOString().slice(0, 16).replace('T', ' ');
    currentSessionId = await createSession(defaultName);
    await renderSessionsList();
}

async function replaySession(id) {
    closeSessionsMenu();
    var messages = await getSessionMessages(id);
    var lines = messages.map(function(m) { return m.line; });
    replayFromSessionMessages(lines);
    openLogMenu();
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

    // Not implemented yet
    document.getElementById("missionplannerlink").style.display = "none";
    document.getElementById("senduavcommandlink").style.display = "none";
    document.getElementById("inavsettingslink").style.display = "none";

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

function openLogMenu()
{
    document.getElementById("logMenu").style.width = "100%";
}

function closeLogMenu() {
    document.getElementById("logMenu").style.width = "0";
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
document.getElementById("navBrokerSettings").addEventListener("click", openBrokerSettings);
document.getElementById("uisettingslink").addEventListener("click", openUISettings);
document.getElementById("navKeepAwake").addEventListener("click", keepScreenAwake);
document.getElementById("navLogOptions").addEventListener("click", openLogMenu);
document.getElementById("navRefreshApp").addEventListener("click", reloadApplication);
document.getElementById("btSaveBrokerSettings").addEventListener("click", saveBrokerSettings);
document.getElementById("btResetBrokerSettings").addEventListener("click", resetBrokerSettings);
document.getElementById("closeBrokerSettings").addEventListener("click", closeBrokerSettings);
document.getElementById("closeUISettings").addEventListener("click", closeUISettings);
document.getElementById("btSaveUISettings").addEventListener("click", saveUISettings);
document.getElementById("closeLogMenu").addEventListener("click", closeLogMenu);
document.getElementById("navSaveLog").addEventListener("click", savemqttlog);
document.getElementById("navReplayLog").addEventListener("click", replaymqttlog);
document.getElementById("btStopReplay").addEventListener("click", stopreplaymqttlog);
document.getElementById("navSessionsMenu").addEventListener("click", openSessionsMenu);
document.getElementById("closeSessionsMenu").addEventListener("click", closeSessionsMenu);
document.getElementById("btRenameSession").addEventListener("click", renameCurrentSession);
document.getElementById("btNewSession").addEventListener("click", newSession);

// Modules are deferred — use DOMContentLoaded instead of window.onload
// to guarantee the handler runs even if the page loaded before the module executed.
window.addEventListener("DOMContentLoaded", async function() {
    // Initialize flight session storage
    try {
        await openDB();
        var openSession = await getOpenSession();
        if (openSession) {
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

    MQTTconnect();
    renderEFIS(data);
    updateDataView(data);
    drawAircraftOnMap(data);

    getUserLocation();
    drawUserOnMap(data);

    var timerEFIS = setInterval(function(){
        estimateEfis();
        renderEFIS(data);
    }, pageSettings.efisRefreshInterval);

    var timerMapAndData = setInterval(function(){
        toggleBlinkFast();

        estimatePosition();

        drawAircraftOnMap(data);
        drawAircraftPathOnMap(data);
        drawCourseLineOnMap(data);
        updateDataView(data);
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
        drawUserOnMap(data);

        if(user_moved_map == true)
            setUserMovedMap(false);
        else
            centerMap(data);

        if(data.waypointCount > 0 && data.isCurrentMissionElevationSet == false
        && data.isWaypointMissionValid == 1 && updatingWpAltitudes == false
        && data.waypointCount == (data.currentMissionWaypoints.length - 1))
        {
            console.log("Trying to get WP Elevation data...");
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
