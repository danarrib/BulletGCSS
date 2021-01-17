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

// Setup Sidebar
function openNav() {
    // If App is running standalone, then don't show the option to install
    document.getElementById("installhomelink").style.display = isRunningStandalone() ? "none" : "";

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

function closeBrokerSettings() {
    document.getElementById("brokerSettings").style.width = "0";
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
    }, 500); // 33 = 30fps, 66 = 15fps, 100 = 10fps, 250 = 4fps, 500 = 2fps

    var timerOneSecond = setInterval(function(){ 
        // Update Flight Time and Power Time
        data.powerTime++;
        if(data.uavIsArmed)
            data.flightTime++;
            
    }, 1000);

    var timerLowPriorityTasks = setInterval(function(){ 
        drawMissionOnMap(data);
        drawHomeOnMap(data);
        drawUserOnMap(data);

        // Center map
        if(user_moved_map == true)
            user_moved_map = false; // Set it false so the next time this routine will move the map
        else
            centerMap(data);

    }, 5000);
    

    var timerBlinkSlow = setInterval(function(){ 
        blinkSlowSwitch = !blinkSlowSwitch;
    }, 2000);

}