var blinkSlowSwitch = true;
var blinkFastSwitch = true;

function openGoogleMaps(lat, lon){
    // If it's an iPhone..
    if( (navigator.platform.indexOf("iPhone") != -1) 
        || (navigator.platform.indexOf("iPod") != -1)
        || (navigator.platform.indexOf("iPad") != -1))
        //window.open("maps://www.google.com/maps/dir/?api=1&travelmode=driving&layer=traffic&destination=" + lat + "," + lon + "");
        window.open("http://maps.apple.com/?q=" + lat + "," + lon + "");
    else
        window.open("https://www.google.com/maps/dir/?api=1&travelmode=driving&layer=traffic&destination=" + lat + "," + lon + "");
}

function secondsToNiceTime(seconds) 
{
    if(seconds > 60 * 60 * 24) // More than one day means never
        return "Never";
    
    var minutes = 0;
    var hours = 0;

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
    if(hours>0)
        ret += hours + "h ";

    if(minutes>0)
        ret += minutes + "m ";
    
    ret += seconds + "s";
    return ret;
}

function centimetersToNiceDistance(centimeters)
{
    var meters = parseInt(centimeters / 100);
    if(meters > 999)
        return (meters / 1000).toFixed(1) + " km";
    else
        return meters + "m";
}

function getConnectionIcon()
{
    var dtNow = new Date();
    var timeSinceLastMessage = parseInt((dtNow - lastMessageDate) / 1000);
    var ret = "";

    if(!mqttConnected)
        ret = "broken";
    if(timeSinceLastMessage <= 10)
        ret = "ok";
    else if (timeSinceLastMessage <= 30)
        ret = "attention";
    else
        ret = "error";

    return "img/connection_" + ret + ".png";
}

function getCellSignalIcon(bars)
{
    var ret = "broken";
    if(bars==1)
        ret = "poor";
    if(bars==2)
        ret = "good";
    if(bars==3)
        ret = "excelent";

    return "img/cell_" + ret + ".png";
}

function getRadioIcon(signalPercent)
{
    var ret = "broken";
    if(signalPercent >= 10 && signalPercent < 30)
        ret = "bad";
    else if(signalPercent >= 30 && signalPercent < 50)
        ret = "poor";
    else if(signalPercent >= 40)
        ret = "ok";

    return "img/rx_" + ret + ".png";
}

function getBatteryIcon(batteryPercent)
{
    var ret = "0";
    if(batteryPercent >= 5 && batteryPercent <= 20)
        ret = "20";
    else if(batteryPercent > 20 && batteryPercent <= 40)
        ret = "40";
    else if(batteryPercent > 40 && batteryPercent <= 60)
        ret = "60";
    else if(batteryPercent > 60 && batteryPercent <= 80)
        ret = "80";
    else if(batteryPercent > 80)
        ret = "100";

    return "img/battery_" + ret + ".png";
}

function getGPSIcon(sats, hdop)
{
    var ret = "broken";
    if(hdop != 0 && hdop < 5) 
    {
        if(sats <= 7)
            ret = "bad";
        else if(sats >=8 && sats <= 10)
            ret = "poor";
        else if(sats >= 11)
            ret = "ok";
    }
    return "img/gps_" + ret + ".png";
}

function getHardwareHealthyIcon(isHwHealthy)
{
    var ret = "error";
    if(isHwHealthy == 1)
        ret = "ok";

    return "img/health_" + ret + ".png";
}

function updateDataView(data)
{
    var PlusCodeCoordinates = OpenLocationCode.encode(data.gpsLatitude, data.gpsLongitude);
    
    var armedClass = "color-normal";
    if(data.uavIsArmed == 1)
        armedClass = "color-ok";

    var gps3DClass = "color-danger";
    if(data.gps3DFix == 1)
        gps3DClass = "color-ok";

    var wpMissionClass = "color-normal";
    var wpMissionText = "Not loaded";
    if(data.waypointCount > 0)
    {
        wpMissionText = data.currentWaypointNumber + ' / ' + data.waypointCount;
        if(data.isWaypointMissionValid == 1)
            wpMissionClass = "color-ok";
        else
            wpMissionClass = "color-danger";
    }

    var activeFlightMode = data.flightMode;
    if(data.isFailsafeActive == 1)
        if(blinkSlowSwitch)
            activeFlightMode = "!FS!";

    if(data.isFailsafeActive == 1 && blinkFastSwitch)
        activeFlightMode = "";

    document.getElementById("waipointPlaceHolder").innerHTML = wpMissionText;
    document.getElementById("waipointPlaceHolder").className = wpMissionClass;

    document.getElementById("cellIcon").src = getCellSignalIcon(data.cellSignalStrength);
    document.getElementById("radioIcon").src = getRadioIcon(data.rssiPercent);
    document.getElementById("batteryIcon").src = getBatteryIcon(data.fuelPercent);
    document.getElementById("connectionIcon").src = getConnectionIcon();
    document.getElementById("gpsIcon").src = getGPSIcon(data.gpsSatCount, data.gpsHDOP);
    document.getElementById("hardwareHealthIcon").src = getHardwareHealthyIcon(data.isHardwareHealthy);

    document.getElementById("activeModePlaceHolder").innerHTML = activeFlightMode;
    document.getElementById("activeModePlaceHolder").className = armedClass;

    document.getElementById("coordinatesPlaceHolder").innerHTML = '<a style="cursor: pointer; color: #00BFFF;" onclick="openGoogleMaps(' + data.gpsLatitude + ',' + data.gpsLongitude + ');" ontouchstart="openGoogleMaps(' + data.gpsLatitude + ',' + data.gpsLongitude + ');">' + PlusCodeCoordinates + '</a>';
    document.getElementById("homeDistancePlaceHolder").innerHTML = centimetersToNiceDistance(data.homeDistance);

    document.getElementById("gpsInfoPlaceHolder").innerHTML = data.gpsSatCount + ' Sats <font class="smalltext">[' + data.gpsHDOP + ' hdop]</font>';
    document.getElementById("gpsInfoPlaceHolder").className = gps3DClass;

    document.getElementById("aziElevPlaceHolder").innerHTML = data.azimuth + 'º / ' + data.elevation + 'º';
    document.getElementById("callsignPlaceHolder").innerHTML = data.callsign;

    document.getElementById("ampDrawPlaceHolder").innerHTML = data.currentDraw + " A";
    document.getElementById("mAhUsedPlaceHolder").innerHTML = data.capacityDraw + " mAh";
    document.getElementById("batteryVoltagePlaceHolder").innerHTML = data.batteryVoltage + "V - " + data.battCellVoltage + "V";
    document.getElementById("rssiPlaceHolder").innerHTML = data.rssiPercent + " %";

    //var onFlightTimeText = data.powerTime + " / " + data.flightTime;
    //document.getElementById("onFlightTimePlaceHolder").innerHTML = onFlightTimeText;

}