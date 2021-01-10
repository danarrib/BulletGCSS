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

function enableWakeLock() {
    noSleep.enable();
    getUserLocation();
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
        updateDataView(data);
    }, 500); // 33 = 30fps, 66 = 15fps, 100 = 10fps, 250 = 4fps, 500 = 2fps

    var timerCenterMap = setInterval(function(){ 
    }, 5000);

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