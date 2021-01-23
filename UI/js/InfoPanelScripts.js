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

function bearing(startLat, startLng, destLat, destLng){
    startLat = AngleToRadians(startLat);
    startLng = AngleToRadians(startLng);
    destLat = AngleToRadians(destLat);
    destLng = AngleToRadians(destLng);

    y = Math.sin(destLng - startLng) * Math.cos(destLat);
    x = Math.cos(startLat) * Math.sin(destLat) -
            Math.sin(startLat) * Math.cos(destLat) * Math.cos(destLng - startLng);
    brng = Math.atan2(y, x);
    brng = RadiansToAngle(brng);
    return (brng + 360) % 360;
}

// Horizon is 0 degree, Up is 90 degree
function getVerticalBearing(fromLat, fromLon, fromAlt, toLat, toLon, toAlt, currentElevation) {
    fromLat = AngleToRadians(fromLat);
    fromLon = AngleToRadians(fromLon);
    toLat = AngleToRadians(toLat);
    toLon = AngleToRadians(toLon);

    let fromECEF = getECEF(fromLat, fromLon, fromAlt);
    let toECEF = getECEF(toLat, toLon, toAlt);
    let deltaECEF = getDeltaECEF(fromECEF, toECEF);

    let d = (fromECEF[0] * deltaECEF[0] + fromECEF[1] * deltaECEF[1] + fromECEF[2] * deltaECEF[2]);
    let a = ((fromECEF[0] * fromECEF[0]) + (fromECEF[1] * fromECEF[1]) + (fromECEF[2] * fromECEF[2]));
    let b = ((deltaECEF[0] * deltaECEF[0]) + (deltaECEF[2] * deltaECEF[2]) + (deltaECEF[2] * deltaECEF[2]));
    let elevation = RadiansToAngle(Math.acos(d / Math.sqrt(a * b)));
    elevation = 90 - elevation;

    return elevation - currentElevation;
}

function getDeltaECEF(from, to) {
    let X = to[0] - from[0];
    let Y = to[1] - from[1];
    let Z = to[2] - from[2];

    return [X, Y, Z];
}

function getECEF(lat, lon, alt) {
    let radius = 6378137;
    let flatteningDenom = 298.257223563;
    let flattening = 0.003352811;
    let polarRadius = 6356752.312106893;

    let asqr = radius * radius;
    let bsqr = polarRadius * polarRadius;
    let e = Math.sqrt((asqr-bsqr)/asqr);
    // let eprime = Math.sqrt((asqr-bsqr)/bsqr);

    let N = getN(radius, e, lat);
    let ratio = (bsqr / asqr);

    let X = (N + alt) * Math.cos(lat) * Math.cos(lon);
    let Y = (N + alt) * Math.cos(lat) * Math.sin(lon);
    let Z = (ratio * N + alt) * Math.sin(lat);

    return [X, Y, Z];
}

function getN(a, e, latitude) {
    let sinlatitude = Math.sin(latitude);
    let denom = Math.sqrt(1 - e * e * sinlatitude * sinlatitude);
    return a / denom;
}








function metersToNiceDistance(meters)
{
    if(uiElementsUnits.distanceUnit == "km")
    {
        if(meters > 9999)
            return (meters / 1000).toFixed(1) + " km";
        else
            return meters + " m";
    }
    else if(uiElementsUnits.distanceUnit == "mi")
    {
        var miles = meters * uiElementsUnits.distanceMetersToMilesFactor;
        if(miles < 0.25)
            return (meters * uiElementsUnits.distanceMetersToFeetFactor).toFixed(0) + " ft";
        else
            return miles.toFixed(1) + " mi";
    }
    else if(uiElementsUnits.distanceUnit == "nm")
    {
        var nMiles = meters * uiElementsUnits.distanceMetersToNauticalMilesFactor;
        return nMiles.toFixed(1) + " Nm";
    }
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

    // Calculate Azimuth and Elevation using User Location. If it's not available, use Home location
    if(hasUserLocation) 
    {
        data.azimuth = bearing(data.userLatitude, data.userLongitude, data.gpsLatitude, data.gpsLongitude).toFixed(0);
        document.getElementById("aziElevPlaceHolder").className = "color-ok";
        if(data.userAltitudeSL !== null)
        {
            document.getElementById("aziElevPlaceHolder").innerHTML = data.azimuth + 'º / ' + data.elevation + 'º';
            data.elevation = getVerticalBearing(data.userLatitude, data.userLongitude, data.userAltitudeSL, data.gpsLatitude, data.gpsLongitude, data.altitudeSeaLevel, 0).toFixed(0);
        }
        else
            document.getElementById("aziElevPlaceHolder").innerHTML = data.azimuth + 'º / N/A';
    }
    else if(data.homeLatitude != 0 && data.homeLongitude != 0)
    {
        data.azimuth = bearing(data.homeLatitude, data.homeLongitude, data.gpsLatitude, data.gpsLongitude).toFixed(0);
        data.elevation = getVerticalBearing(data.homeLatitude, data.homeLongitude, data.homeAltitudeSL, data.userAltitudeSL, data.gpsLatitude, data.gpsLongitude, data.altitudeSeaLevel, 0).toFixed(0);
        document.getElementById("aziElevPlaceHolder").className = "color-normal";
        document.getElementById("aziElevPlaceHolder").innerHTML = data.azimuth + 'º / ' + data.elevation + 'º';
    }
    else
    {
        data.azimuth = bearing(data.homeLatitude, data.homeLongitude, data.gpsLatitude, data.gpsLongitude).toFixed(0);
        document.getElementById("aziElevPlaceHolder").className = "color-normal";
        document.getElementById("aziElevPlaceHolder").innerHTML = "";
    }

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
    
    
    document.getElementById("homeDistancePlaceHolder").innerHTML = metersToNiceDistance(data.homeDistance);

    document.getElementById("gpsInfoPlaceHolder").innerHTML = data.gpsSatCount + ' Sats <font class="smalltext">[' + data.gpsHDOP + ' hdop]</font>';
    document.getElementById("gpsInfoPlaceHolder").className = gps3DClass;

    document.getElementById("callsignPlaceHolder").innerHTML = data.callsign.length > 0 ? data.callsign : "&nbsp;";

    if(uiElementsUnits.currentUnit == "a")
    {
        document.getElementById("ampDrawPlaceHolder").innerHTML = data.currentDraw + " A";
        document.getElementById("ampDrawLabel").innerHTML = "Curr Draw";
        
    }
    else if(uiElementsUnits.currentUnit == "w")
    {
        document.getElementById("ampDrawPlaceHolder").innerHTML = (data.currentDraw * data.batteryVoltage).toFixed(0) + " W";
        document.getElementById("ampDrawLabel").innerHTML = "Pwr Draw";
    }

    if(uiElementsUnits.capacityUnit == "mah")
    {
        document.getElementById("mAhUsedPlaceHolder").innerHTML = data.capacityDraw + " mAh";
        document.getElementById("capacityLabel").innerHTML = "mAh used";
    }
    else if(uiElementsUnits.capacityUnit == "mwh")
    {
        if(data.mWhDraw < 1000) {
            document.getElementById("mAhUsedPlaceHolder").innerHTML = data.mWhDraw + " mWh";
            document.getElementById("capacityLabel").innerHTML = "mWh used";
        }
        else
        {
            document.getElementById("mAhUsedPlaceHolder").innerHTML = (data.mWhDraw / 1000).toFixed(1) + " Wh";
            document.getElementById("capacityLabel").innerHTML = "Wh used";
        }
    }

    document.getElementById("throttlePlaceHolder").innerHTML = data.throttlePercent + " %";
    if(data.isAutoThrottleActive)
        document.getElementById("throttlePlaceHolder").className = "color-warning";
    else
        document.getElementById("throttlePlaceHolder").className = "color-normal";


    var mAhPerKm = 0;
    if(data.groundSpeed > 0 && data.currentDraw > 0) 
    {
        mAhPerKm = ((data.currentDraw / 60) * 1000 ) / ( (data.groundSpeed * efis.SpeedUnitFactor) / 60);
        var mAhPerMi = mAhPerKm * 1.60934;

        if(uiElementsUnits.efficiencyUnit == "mahkm")
        {
            document.getElementById("efficiencyPlaceHolder").innerHTML = mAhPerKm.toFixed(0) + " mAh/Km";
        }
        else if(uiElementsUnits.efficiencyUnit == "mwhkm")
        {
            document.getElementById("efficiencyPlaceHolder").innerHTML = (mAhPerKm * data.batteryVoltage).toFixed(0) + " mWh/Km";
        }
        else if(uiElementsUnits.efficiencyUnit == "mahmi")
        {
            document.getElementById("efficiencyPlaceHolder").innerHTML = mAhPerMi.toFixed(0) + " mAh/mi";
        }
        else if(uiElementsUnits.efficiencyUnit == "mwhmi")
        {
            document.getElementById("efficiencyPlaceHolder").innerHTML = (mAhPerMi * data.batteryVoltage).toFixed(0) + " mWh/mi";
        }
    }
    else
    {
        document.getElementById("efficiencyPlaceHolder").innerHTML = "--";
    }


    document.getElementById("rssiPlaceHolder").innerHTML = data.rssiPercent + " %";
    if(data.rssiPercent <= 20)
        document.getElementById("rssiPlaceHolder").className = "color-danger";
    else if(data.rssiPercent <= 60)
        document.getElementById("rssiPlaceHolder").className = "color-warning";
    else
        document.getElementById("rssiPlaceHolder").className = "color-ok";

    document.getElementById("batteryVoltagePlaceHolder").innerHTML = data.batteryVoltage + "V - " + data.battCellVoltage + "V";
    if(data.fuelPercent <= 20)
        document.getElementById("batteryVoltagePlaceHolder").className = "color-danger";
    else if(data.fuelPercent <= 60)
        document.getElementById("batteryVoltagePlaceHolder").className = "color-warning";
    else
        document.getElementById("batteryVoltagePlaceHolder").className = "color-ok";

    if(blinkSlowSwitch || data.uavIsArmed)
    {
        document.getElementById("flightTimeLabel").innerHTML = "Flt time";
        document.getElementById("flightTimePlaceHolder").innerHTML = secondsToNiceTime(data.flightTime);
    }
    else
    {
        document.getElementById("flightTimeLabel").innerHTML = "Uptime";
        document.getElementById("flightTimePlaceHolder").innerHTML = secondsToNiceTime(data.powerTime);
    }

    // Status Text - When Slowblink is true, show UI messages. When it's false, show aircraft messages
    if(blinkSlowSwitch)
    {
        // UI messages
        if(blinkFastSwitch || 1==1)
        {
            var dtNow = new Date();
            var timeSinceLastMessage = parseInt((dtNow - lastMessageDate) / 1000);

            if(!mqttConnected)
                document.getElementById("statusPlaceHolder").innerHTML = "MQTT Broker not connected";
            else if(timeSinceLastMessage >= 5)
                document.getElementById("statusPlaceHolder").innerHTML = "Last message: " + secondsToNiceTime(timeSinceLastMessage);
            else if(updatingWpAltitudes)
                document.getElementById("statusPlaceHolder").innerHTML = "Fetching WP elevation data...";
            else
                document.getElementById("statusPlaceHolder").innerHTML = "&nbsp;";
        }
        else
        {
            document.getElementById("statusPlaceHolder").innerHTML = "&nbsp;";
        }
    }
    else
    {
        // Aircraft messages
        if(blinkFastSwitch)
        {
            if(data.navState == 2)
                document.getElementById("statusPlaceHolder").innerHTML = "En route to home";
            else if(data.navState == 5)
                document.getElementById("statusPlaceHolder").innerHTML = "En route to WP " + data.currentWaypointNumber + "/" + data.waypointCount;
            else if(data.navState == 9)
                document.getElementById("statusPlaceHolder").innerHTML = "Landing";
            else if(data.navState == 14)
                document.getElementById("statusPlaceHolder").innerHTML = "Emergency landing";
            else
                document.getElementById("statusPlaceHolder").innerHTML = "&nbsp;";
        }
        else
        {
            if(data.isFailsafeActive)
                document.getElementById("statusPlaceHolder").innerHTML = "!RX RC Link lost!";
            else
                document.getElementById("statusPlaceHolder").innerHTML = "&nbsp;";
        }
    }



}