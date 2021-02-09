var efis = {
    verticalFOV: 60, // Degrees
    horizontalFOV: 50, // Degrees
    AHIStepsDeg: 10, // Degrees
    SpeedUnitLabel: 'km/h',
    SpeedUnitFactor: 0.036, // value that * m/s result in speed on Unit desired (0.036 = m/s to km/h)
    SpeedFOV: 50, // value is on Speed unit (ex: 50 km/h)
    SpeedSteps: 10, // Each line on the Speed scrolling bar
    AltitudeFOV: 50, // value is on Altitude Unit (ex: 50 m)
    AltitudeSteps: 10, // Each line on the Altitude Scrolling bar
    AltitudeUnitLabel: 'mts',
    AltitudeUnitFactor: 100, // value that * cm result in altitude on Unit desired (100 = cm to m)
    AltitudeSmallTextNumber: 2, // Last X numbers of altitude will be smaller than the remaining first ones
    VerticalSpeedUnitFactor: 100, // value that * cm/s result in the vertical speed unit desired (100 = cm/s to m/s)
    VerticalSpeedUnitLabel: 'm/s', 
    VerticalSpeedSteps: 2, // Each line on the Vertical Speed bar
    VerticalSpeedFOV: 8, // from 0 to this number (and negative too)
    qtyBlocks: 18, // How many horizontal and vertical chuncks the hud will be divided
    fontSizeFactor: 4.5, // (HUD Height / qtyBlocks) * fontSizeFactor
    horizonSizeFactor: 8, // Greater number, shorter AHI lines
    hudView: document.getElementById("hudview"),
    efisCanvas: document.getElementById("cvsEFIS"),
    DefaultFont: 'Ubuntu',
};

function P(x, y) { return {x: x, y: y}; }		

function getParallelSegment(A, B, d, side) {
    // --- Return a line segment parallel to AB, d pixels away
    var dx = A.x - B.x,
        dy = A.y - B.y,
        dist = Math.sqrt(dx*dx + dy*dy) / 2;
    side = side || 1;
    dx *= side * d / dist;
    dy *= side * d / dist;
    return [P(A.x + dy, A.y - dx), P(B.x + dy, B.y - dx)];
}

function sq(x) { return x*x ;}

function buildUnitVector(p1, p2, uVect) {
    uVect.x = (p2.x - p1.x);
    uVect.y = (p2.y - p1.y );
    var vectorNorm = Math.sqrt( sq(uVect.x)+sq(uVect.y) );
    uVect.x/=vectorNorm;
    uVect.y/=vectorNorm;
}

function drawProgressBar(ctx, text, x, y, w, h, fillPercent, color, font, borderSize) {
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = borderSize;
    ctx.rect(x, y, w, h);
    ctx.fillRect(x, y, ((w / 100) * fillPercent), h);

    drawTextWithShadow(ctx, text, x + (w / 2), y + (h / 1.5) + borderSize, font, 'center', 'white', 'black', borderSize);

    ctx.stroke();
    ctx.closePath();

}

function drawTextWithShadow(ctx, text, x, y, font, textAlign, textColor, shadowColor, shadowDistance) {
    ctx.font = font;
    ctx.textAlign = textAlign;
    ctx.fillStyle = shadowColor;
    efis.efisContext.fillText(text, x + shadowDistance, y + shadowDistance);
    ctx.fillStyle = textColor;
    ctx.fillText(text, x, y);
}

function drawShorterLine(ctx, p1, p2, a, b) {
    buildUnitVector(p1, p2, unitVector);
    sp1.x = p1.x + unitVector.x * a;
    sp1.y = p1.y + unitVector.y * a;
    sp2.x = p2.x - unitVector.x * b;
    sp2.y = p2.y - unitVector.y * b;
    ctx.beginPath();
    ctx.moveTo(sp1.x, sp1.y);
    ctx.lineTo(sp2.x, sp2.y);
    ctx.stroke();
}
var unitVector = { x:0, y:0 }, sp1 = {x:0,y:0},	sp2 = {x:0, y:0 } ; 		

function lineDistance( point1, point2 ){
    var xs = 0;
    var ys = 0;

    xs = point2.x - point1.x;
    xs = xs * xs;

    ys = point2.y - point1.y;
    ys = ys * ys;

    return Math.sqrt( xs + ys );
}

function correctPitchAngle(data, line, pitchDegPixels) {
    var correctedPitchAngle = data.pitchAngle;
    if(correctedPitchAngle > 90) {
        // Aircraft is upside down. Start decreasing pitch angle and flip horizon
        correctedPitchAngle = 180 - correctedPitchAngle;
    } else if(correctedPitchAngle < -90) {
        // Aircraft is upside down. Start decreasing pitch angle and flip horizon
        correctedPitchAngle = 180 + correctedPitchAngle;
    }
    
    // On INAV, nose down means positive number and nose up negative. Fix it now.
    correctedPitchAngle = correctedPitchAngle * -1;
    
    var rollAngleRad = data.rollAngle * (Math.PI / 180) * -1;
    var pitchAngleRad = correctedPitchAngle * (Math.PI / 180);
    var cosRoll = Math.cos(rollAngleRad);
    var sinRoll = Math.sin(rollAngleRad);
    var cosPitch = Math.cos(pitchAngleRad);
    var sinPitch = Math.sin(pitchAngleRad);
    
    line[0].y += pitchDegPixels * cosRoll * correctedPitchAngle;
    line[1].y += pitchDegPixels * cosRoll * correctedPitchAngle;

    line[0].x += pitchDegPixels * -sinRoll * correctedPitchAngle;
    line[1].x += pitchDegPixels * -sinRoll * correctedPitchAngle;

}

function drawBox(ctx, coordinates, size, borderSize, borderColor, boxColor) {
    ctx.fillStyle = borderColor;
    ctx.fillRect(coordinates.x, coordinates.y, size.x, size.y);
    ctx.fillStyle = boxColor;
    ctx.fillRect(coordinates.x + borderSize, coordinates.y + borderSize, size.x - (borderSize * 2), size.y - (borderSize * 2));
}

function pad(num, size) {
    var s = num+"";
    while (s.length < size) s = "0" + s;
    return s;
}

function drawBoxWithTextAbsPosition(ctx, text, textColor, textSize, textAlign, position, size, placeholder, smallNumbers = 0) {
    var bigText = text;
    var smallText = bigText.toString().slice(-smallNumbers);
    var smallTextWidth = 0;
    if(smallNumbers>0) {
        ctx.font = (textSize/2) + 'px ' + efis.DefaultFont;
        smallTextWidth = ctx.measureText(smallText).width;
        bigText = text.toString().substring(0, text.toString().length - smallNumbers);
        /*
        console.log("Small: " + smallText + " - " + smallTextWidth);
        console.log("Big: " + bigText);
        */
    }

    ctx.font = textSize + 'px ' + efis.DefaultFont;
    drawBox(ctx, position, size, 2, textColor, 'black');
    ctx.fillStyle = textColor;
    ctx.textAlign = textAlign;
    var textStartPosition = 0;

    if(textAlign=='right')
        textStartPosition = size.x / 1.1;
    else if(textAlign=='center')
        textStartPosition = size.x / 2;

    if(smallNumbers>0) {
        ctx.font = (textSize/2) + 'px ' + efis.DefaultFont;
        ctx.fillText(smallText, position.x + textStartPosition, position.y + (textSize / 1.5));
    }
    ctx.font = textSize + 'px ' + efis.DefaultFont;
    ctx.fillText(bigText, position.x + textStartPosition - smallTextWidth, position.y + (textSize / 1.2));
}

function drawBoxWithText(ctx, text, textColor, textSize, textAlign, position, placeholder) {
    ctx.font = textSize + 'px ' + efis.DefaultFont;
    var boxWidth = ctx.measureText(placeholder).width * 1.1;

    if(textAlign=='right')
        position.x -= boxWidth;
    else if(textAlign=='center')
        position.x -= boxWidth / 2;

    drawBoxWithTextAbsPosition(ctx, text, textColor, textSize, textAlign, position, P(boxWidth, textSize), placeholder)
}

function drawEfisHeadingIndicator() {
    var numberY = efis.efisHeight - (efis.fontSize);
    var lineY1 = numberY + (efis.fontSize);
    var lineY2 = numberY + (efis.fontSize * 0.1);

    for(i=0; i <= (359 + efis.horizontalFOV); i+=efis.AHIStepsDeg) {
        var startingPosX = (efis.efisWidth / 2) - (data.heading * (efis.efisWidth / efis.horizontalFOV));
        var numberX = startingPosX + (i * (efis.efisWidth / efis.horizontalFOV) );
        var numberXNeg = startingPosX + (-i * (efis.efisWidth / efis.horizontalFOV) );
        var nAngle = i;
        if(nAngle > 359)
            nAngle = nAngle - 360;
        var txtAngle = pad(nAngle, 3);
        var txtAngleNeg = pad(360 - i, 3);

        if(i==0 || i== 360) {
            txtAngle = "N";
            txtAngleNeg = "N";
        }

        if(i==90) {
            txtAngle = "E";
            txtAngleNeg = "W";
        }

        if(i==180) {
            txtAngle = "S";
            txtAngleNeg = "S";
        }

        if(i==270) {
            txtAngle = "W";
            txtAngleNeg = "E";
        }

        efis.efisContext.beginPath();
        efis.efisContext.lineWidth = 1 * window.devicePixelRatio;
        efis.efisContext.fillStyle = 'white';
        efis.efisContext.font = efis.fontSize + 'px ' + efis.DefaultFont;
        efis.efisContext.textAlign = "center";
        efis.efisContext.fillText(txtAngle, numberX, numberY);
        efis.efisContext.moveTo(numberX, lineY1);
        efis.efisContext.lineTo(numberX, lineY2);
        efis.efisContext.moveTo(numberX + ((efis.efisWidth / efis.horizontalFOV) * (efis.AHIStepsDeg /2)), lineY1);
        efis.efisContext.lineTo(numberX + ((efis.efisWidth / efis.horizontalFOV) * (efis.AHIStepsDeg /2)), lineY2);
        if(i>0) {
            efis.efisContext.fillText(txtAngleNeg, numberXNeg, numberY);
            efis.efisContext.moveTo(numberXNeg, lineY1);
            efis.efisContext.lineTo(numberXNeg, lineY2);
            efis.efisContext.moveTo(numberXNeg + ((efis.efisWidth / efis.horizontalFOV) * (efis.AHIStepsDeg /2)), lineY1);
            efis.efisContext.lineTo(numberXNeg + ((efis.efisWidth / efis.horizontalFOV) * (efis.AHIStepsDeg /2)), lineY2);
        }
        efis.efisContext.strokeStyle = 'white';
        efis.efisContext.stroke();
        efis.efisContext.closePath();
    }

    efis.efisContext.beginPath();
    var headingDisplay = data.heading;
    if(headingDisplay < 0)
        headingDisplay = 360 + headingDisplay;
    headingDisplay = pad(parseInt(headingDisplay), 3);

    drawBoxWithText(efis.efisContext, headingDisplay + 'º', 'white', efis.fontSize * 2, 'center', P((efis.efisWidth / 2), efis.efisHeight - (efis.fontSize * 2)), '000º');
    efis.efisContext.closePath();
}

function drawEfisHorizonGroundSky() {
    // Render Sky and Ground
    // Make the horizon line angle
    efis.horizonX1 = (efis.efisWidth / 2);
    efis.horizonY1 = (efis.efisHeight / 2);
    efis.rollAngleRad = (data.rollAngle * (Math.PI / 180)) * -1;
    efis.horizonX2 = parseInt(efis.horizonX1 + (efis.lineLength * Math.cos(efis.rollAngleRad)));
    efis.horizonY2 = parseInt(efis.horizonY1 + (efis.lineLength * Math.sin(efis.rollAngleRad)));
    efis.horizonX1 = efis.efisWidth - efis.horizonX2;
    efis.horizonY1 = efis.efisHeight - efis.horizonY2;
    var HorizonLine = [P(efis.horizonX1, efis.horizonY1), P(efis.horizonX2, efis.horizonY2)];
    
    // Apply pitch to the Horizon line
    correctPitchAngle(data, HorizonLine, efis.pitchDegPixels);
    
    // Draw Sky
    var SkyLine = getParallelSegment(P(HorizonLine[0].x, HorizonLine[0].y), P(HorizonLine[1].x, HorizonLine[1].y), efis.lineLength, -1);
    efis.efisContext.beginPath();
    efis.efisContext.moveTo(SkyLine[0].x, SkyLine[0].y);
    efis.efisContext.lineTo(HorizonLine[0].x, HorizonLine[0].y);
    efis.efisContext.lineTo(HorizonLine[1].x, HorizonLine[1].y);
    efis.efisContext.lineTo(SkyLine[1].x, SkyLine[1].y);
    efis.efisContext.closePath();
    efis.efisContext.fillStyle = '#0086ad';
    efis.efisContext.fill();
    
    // Draw Ground
    var GroundLine = getParallelSegment(P(HorizonLine[0].x, HorizonLine[0].y), P(HorizonLine[1].x, HorizonLine[1].y), efis.lineLength, 1);
    efis.efisContext.beginPath();
    efis.efisContext.moveTo(HorizonLine[0].x, HorizonLine[0].y);
    efis.efisContext.lineTo(GroundLine[0].x, GroundLine[0].y);
    efis.efisContext.lineTo(GroundLine[1].x, GroundLine[1].y);
    efis.efisContext.lineTo(HorizonLine[1].x, HorizonLine[1].y);
    efis.efisContext.closePath();
    efis.efisContext.fillStyle = '#744044';
    efis.efisContext.fill();
    
    // Draw horizon line
    efis.efisContext.beginPath();
    efis.efisContext.moveTo(HorizonLine[0].x, HorizonLine[0].y);
    efis.efisContext.lineTo(HorizonLine[1].x, HorizonLine[1].y);
    efis.efisContext.strokeStyle = 'white';
    efis.efisContext.stroke();
    efis.efisContext.closePath();
}

function drawEfisArtifitialHorizonStepLines() {
    efis.horizonX1 = (efis.efisWidth / 2);
    efis.horizonY1 = (efis.efisHeight / 2);
    efis.horizonX2 = (efis.horizonX1 + ((efis.efisWidth / efis.horizonSizeFactor) * Math.cos(efis.rollAngleRad)));
    efis.horizonY2 = (efis.horizonY1 + ((efis.efisWidth / efis.horizonSizeFactor) * Math.sin(efis.rollAngleRad)));
    efis.horizonX1 = efis.efisWidth - efis.horizonX2;
    efis.horizonY1 = efis.efisHeight - efis.horizonY2;
    var AHILine = [P(efis.horizonX1, efis.horizonY1), P(efis.horizonX2, efis.horizonY2)];
    var halfLineLength = lineDistance(AHILine[0], AHILine[1]) / 4;

    correctPitchAngle(data, AHILine, efis.pitchDegPixels);
    
    for(i=0; i<=180; i+=efis.AHIStepsDeg / 2) {
        
        var StepsRadius = efis.verticalFOV / 3;
        var dataPitchAngle = data.pitchAngle * -1; // Fix INAV inverted pitch angle
        var StepsStart = dataPitchAngle - StepsRadius;
        var StepsFinish = dataPitchAngle + StepsRadius;

        var renderPos = true;
        var renderNeg = true;

        if(StepsStart >= 0 && i < StepsStart)
            renderPos = false;

        if(StepsFinish >= 0 && i > StepsFinish)
            renderPos = false;

        if(StepsStart <= 0 && -i < StepsStart)
            renderNeg = false;

        if(StepsFinish <= 0 && -i > StepsFinish)
            renderNeg = false;

        if(StepsStart >= 0 && StepsFinish > 0)
            renderNeg = false;

        if(StepsStart < 0 && StepsFinish < 0)
            renderPos = false;

        var distanceFromHorizon = (i * efis.pitchDegPixels) / 2;
        var fullLine = (i % efis.AHIStepsDeg == 0);
        
        var AHIStepLine = AHILine;
        var AHIStepLineNeg = AHILine;

        if(i>0) {
            AHIStepLine = getParallelSegment(P(AHILine[0].x, AHILine[0].y), P(AHILine[1].x, AHILine[1].y), distanceFromHorizon, -1);
            AHIStepLineNeg = getParallelSegment(P(AHILine[0].x, AHILine[0].y), P(AHILine[1].x, AHILine[1].y), distanceFromHorizon, 1);
        }

        efis.efisContext.beginPath();
        if(fullLine) {
            if(renderPos) {
                efis.efisContext.moveTo(AHIStepLine[0].x, AHIStepLine[0].y);
                efis.efisContext.lineTo(AHIStepLine[1].x, AHIStepLine[1].y);
            }
            if(i>0) {
                if(renderNeg) {
                    efis.efisContext.moveTo(AHIStepLineNeg[0].x, AHIStepLineNeg[0].y);
                    efis.efisContext.lineTo(AHIStepLineNeg[1].x, AHIStepLineNeg[1].y);
                }
                efis.efisContext.lineWidth = 0.7 * window.devicePixelRatio;
            }
            else{
                efis.efisContext.lineWidth = 1 * window.devicePixelRatio;
            }
            efis.efisContext.strokeStyle = 'white';
            efis.efisContext.stroke();
        }
        else {
            if(renderPos)
                drawShorterLine(efis.efisContext, AHIStepLine[0], AHIStepLine[1], halfLineLength, halfLineLength);

            if(i>0)
                if(renderNeg)
                    drawShorterLine(efis.efisContext, AHIStepLineNeg[0], AHIStepLineNeg[1], halfLineLength, halfLineLength);
        }
        efis.efisContext.closePath();
        
        // Writing angles next to the lines
        if(fullLine) {
            var txtAngle = i;
            if(txtAngle > 90) 
                txtAngle = 90 - (txtAngle - 90);
            
            if(renderPos) {
                efis.efisContext.save();
                efis.efisContext.translate(AHIStepLine[0].x, AHIStepLine[0].y);
                efis.efisContext.rotate(efis.rollAngleRad);
                efis.efisContext.beginPath();
                efis.efisContext.fillStyle = 'white';
                efis.efisContext.font = (efis.fontSize * 0.7) + 'px ' + efis.DefaultFont;
                efis.efisContext.textAlign = "center";
                efis.efisContext.fillText(txtAngle, efis.sizeFactorEfis * 2 * -1, 3);
                efis.efisContext.fillText(txtAngle, (efis.sizeFactorEfis * 2) + (efis.efisWidth / (efis.horizonSizeFactor / 2)), 3);
                efis.efisContext.closePath();
                efis.efisContext.restore();
            }
            if(i>0) {
                if(renderNeg) {
                    efis.efisContext.save();
                    efis.efisContext.translate(AHIStepLineNeg[0].x, AHIStepLineNeg[0].y);
                    efis.efisContext.rotate(efis.rollAngleRad);
                    efis.efisContext.beginPath();
                    efis.efisContext.fillStyle = 'white';
                    efis.efisContext.font = (efis.fontSize * 0.7) + 'px ' + efis.DefaultFont;
                    efis.efisContext.textAlign = "center";
                    efis.efisContext.fillText(-txtAngle, efis.sizeFactorEfis * 2 * -1, 3);
                    efis.efisContext.fillText(-txtAngle, (efis.sizeFactorEfis * 2) + (efis.efisWidth / (efis.horizonSizeFactor / 2)), 3);
                    efis.efisContext.closePath();
                    efis.efisContext.restore();
                }
            }
        }
    }
}

function drawEfisCrosshair() {
    var elementLineWidth = 3 * window.devicePixelRatio;
    var blockStart = 5;
    var blockEnd = 7;
    var elementFont = efis.fontSize + 'px ' + efis.DefaultFont;

    efis.efisContext.beginPath();
    efis.efisContext.lineWidth = elementLineWidth;
    efis.efisContext.fillStyle = 'yellow';
    efis.efisContext.strokeStyle = 'yellow';
    efis.efisContext.moveTo(efis.blockWidth * blockStart, (efis.efisHeight / 2));
    efis.efisContext.lineTo(efis.blockWidth * blockEnd, (efis.efisHeight / 2));
    efis.efisContext.moveTo(efis.blockWidth * blockEnd, (efis.efisHeight / 2) - (elementLineWidth / 2));
    efis.efisContext.lineTo(efis.blockWidth * blockEnd, (efis.efisHeight / 2) + (elementLineWidth * 3));
    efis.efisContext.moveTo(efis.blockWidth * (efis.qtyBlocks - blockStart), (efis.efisHeight / 2));
    efis.efisContext.lineTo(efis.blockWidth * (efis.qtyBlocks - blockEnd), (efis.efisHeight / 2));
    efis.efisContext.moveTo(efis.blockWidth * (efis.qtyBlocks - blockEnd), (efis.efisHeight / 2) - (elementLineWidth / 2));
    efis.efisContext.lineTo(efis.blockWidth * (efis.qtyBlocks - blockEnd), (efis.efisHeight / 2) + (elementLineWidth * 3));
    efis.efisContext.fillRect((efis.efisWidth / 2) - elementLineWidth, (efis.efisHeight / 2) - elementLineWidth, elementLineWidth * 2, elementLineWidth * 2);
    efis.efisContext.stroke();

    drawTextWithShadow(efis.efisContext, (data.pitchAngle * -1).toFixed(1) + 'º', (efis.blockWidth * blockStart) - elementLineWidth, (efis.efisHeight / 2) + (efis.fontSize / 2.5), elementFont, 'right', 'white', 'black', elementLineWidth / 2);

    efis.efisContext.closePath();

}

function drawEfisVerticalSpeed() {
    var elementWidth = efis.blockWidth;
    var elementHeight = efis.efisHeight;
    var elementX = (efis.efisWidth - elementWidth);
    var elementY = 0;
    var elementCenterY = (elementY + elementHeight) / 2;
    var elementFontSize = efis.fontSize * 2;
    var elementSmallFontSize = elementFontSize / 3;
    var verticalSpeedDisplay = data.verticalSpeed.toFixed(1);
    var elementLineStartX = elementX + (elementWidth / 6);
    var elementLineEndX = (elementX + elementWidth) - (elementWidth / 2);
    
    // Draw translucid element space
    efis.efisContext.globalAlpha = 0.5;
    efis.efisContext.fillStyle = 'black';
    efis.efisContext.fillRect(elementX, elementY, elementWidth, elementHeight);
    efis.efisContext.globalAlpha = 1.0;

    // Draw center line
    if(0==1) {
        efis.efisContext.beginPath();
        efis.efisContext.strokeStyle = 'white';
        efis.efisContext.lineWidth = 1 * window.devicePixelRatio;
        efis.efisContext.moveTo(elementX, elementCenterY);
        efis.efisContext.lineTo(elementX + elementWidth, elementCenterY);
        efis.efisContext.stroke();
        efis.efisContext.closePath();
    }

    // Draw numbers and lines
    var maxVerticalSpeed = efis.VerticalSpeedFOV; // m/s

    if(0==1) {
        if(Math.abs(data.verticalSpeed / efis.VerticalSpeedUnitFactor) > maxVerticalSpeed)
            maxVerticalSpeed = Math.ceil(abs(data.verticalSpeed / efis.VerticalSpeedUnitFactor));
    }

    var VerticalSpeedDivisions = efis.VerticalSpeedSteps;
    var textX = (elementX + elementWidth) - (elementWidth / 6);
    var VSPixels = elementHeight / ((maxVerticalSpeed + 1) * 2);
    
    for(i=0; i<=maxVerticalSpeed; i++) {
        if(i % VerticalSpeedDivisions != 0)
            continue;

        efis.efisContext.beginPath();

        var textY = elementCenterY - (i * VSPixels);

        efis.efisContext.moveTo(elementLineStartX, textY);
        efis.efisContext.lineTo(elementLineEndX, textY);


        textY = textY + (elementSmallFontSize / 3);
        efis.efisContext.fillStyle = 'white';
        efis.efisContext.strokeStyle = 'white';
        efis.efisContext.lineWidth = 1 * window.devicePixelRatio;
        efis.efisContext.font = elementSmallFontSize + 'px ' + efis.DefaultFont;
        efis.efisContext.textAlign = "right";
        efis.efisContext.fillText(i, textX, textY);
        if(i>0) {
            textY = elementCenterY + (i * VSPixels);

            efis.efisContext.moveTo(elementLineStartX, textY);
            efis.efisContext.lineTo(elementLineEndX, textY);

            textY = textY + (elementSmallFontSize / 3);
            efis.efisContext.fillText(-i, textX, textY);
        }

        if(i==maxVerticalSpeed) { // last number
            efis.efisContext.moveTo(elementLineStartX, elementCenterY);
            textY = elementCenterY - (i * VSPixels);
            efis.efisContext.lineTo(elementLineStartX, textY);
            textY = elementCenterY + (i * VSPixels);
            efis.efisContext.lineTo(elementLineStartX, textY);
        }

        efis.efisContext.stroke();
        efis.efisContext.closePath();
    }

    // Draw the line for the Vertical Speed value
    efis.efisContext.beginPath();
    efis.efisContext.strokeStyle = 'yellow';
    efis.efisContext.lineWidth = 2 * window.devicePixelRatio;
    var pointerFactor = 0.8;
    
    var vsLineY = elementCenterY - ((data.verticalSpeed / efis.VerticalSpeedUnitFactor) * VSPixels);
    var vsLineY2 = elementCenterY - ((data.verticalSpeed / efis.VerticalSpeedUnitFactor) * VSPixels * pointerFactor);

    if((data.verticalSpeed / efis.VerticalSpeedUnitFactor) > maxVerticalSpeed) {
        vsLineY = elementCenterY - (maxVerticalSpeed * VSPixels);
        vsLineY2 = elementCenterY - (maxVerticalSpeed * VSPixels * pointerFactor);
        efis.efisContext.strokeStyle = 'orange';
    }else if((data.verticalSpeed / efis.VerticalSpeedUnitFactor) < -maxVerticalSpeed) {
        vsLineY = elementCenterY + (maxVerticalSpeed * VSPixels);
        vsLineY2 = elementCenterY + (maxVerticalSpeed * VSPixels * pointerFactor);
        efis.efisContext.strokeStyle = 'orange';
    }

    efis.efisContext.moveTo((elementX + elementWidth), vsLineY2);
    efis.efisContext.lineTo(elementLineStartX, vsLineY);

    efis.efisContext.stroke();
    efis.efisContext.closePath();


    
}

function drawEfisGroundSpeed() {
    var elementWidth = (efis.efisWidth / 6);
    var elementHeight = (efis.efisHeight);
    var elementX = 0;
    var elementY = 0;
    var elementFontSize = efis.fontSize * 2;
    var elementSmallFontSize = elementFontSize / 2;
    var groundSpeedDisplay = data.groundSpeed * efis.SpeedUnitFactor;

    // Draw translucid element space
    efis.efisContext.globalAlpha = 0.2;
    efis.efisContext.fillStyle = 'black';
    efis.efisContext.fillRect(elementX, elementY, elementWidth, elementHeight);
    efis.efisContext.globalAlpha = 1.0;


    // Draw Speed Scroller
    efis.efisContext.lineWidth = 1 * window.devicePixelRatio;
    var startSpeed = parseInt(groundSpeedDisplay - ((efis.SpeedFOV / 2) + efis.SpeedSteps));
    if(startSpeed < 0) startSpeed = 0;
    var endSpeed = parseInt(groundSpeedDisplay + ((efis.SpeedFOV / 2) + efis.SpeedSteps));

    var numberX = elementX + elementWidth;
    var lineX1 = numberX;
    var lineX2 = numberX - ((elementWidth / 8) * 0.9);
    var textX = numberX - (elementWidth / 8);

    for(i=startSpeed; i<=endSpeed;i++) {
        if(i % efis.SpeedSteps == 0) {
            var startingPosY = (elementHeight / 2) - (groundSpeedDisplay * (elementHeight / efis.SpeedFOV));
            var numberY = startingPosY + (i * (elementHeight / efis.SpeedFOV) );
            numberY = elementHeight - numberY;

            efis.efisContext.beginPath();
            efis.efisContext.fillStyle = 'white';
            efis.efisContext.font = elementSmallFontSize + 'px ' + efis.DefaultFont;
            efis.efisContext.textAlign = "right";
            efis.efisContext.fillText(i, textX, numberY + (elementSmallFontSize / 2.5));
            efis.efisContext.moveTo(lineX1, numberY);
            efis.efisContext.lineTo(lineX2, numberY);
            if(i>0) {
                efis.efisContext.moveTo(lineX1, numberY + ((elementHeight / efis.SpeedFOV) * (efis.SpeedSteps /2)));
                efis.efisContext.lineTo(lineX2, numberY + ((elementHeight / efis.SpeedFOV) * (efis.SpeedSteps /2)));
            }
            efis.efisContext.strokeStyle = 'white';
            efis.efisContext.stroke();
            efis.efisContext.closePath();
        }
    }
    
    // Draw box on center of the element
    efis.efisContext.beginPath();
    drawBoxWithTextAbsPosition(efis.efisContext, groundSpeedDisplay.toFixed(0), 'white', elementFontSize, 'right', P(elementX, (elementHeight / 2) - (elementFontSize / 2)), P(elementWidth, elementFontSize), '000');

    // Draw unit
    efis.efisContext.fillStyle = 'white';
    efis.efisContext.font = elementSmallFontSize + 'px ' + efis.DefaultFont;
    efis.efisContext.textAlign = "left";
    efis.efisContext.fillText(efis.SpeedUnitLabel, elementX, (elementHeight / 2) + elementFontSize);
    efis.efisContext.closePath();

}

function drawEfisAltitude() {
    var elementWidth = (efis.efisWidth / 6);
    var elementHeight = (efis.efisHeight);
    var elementX = (efis.efisWidth - elementWidth - efis.blockWidth);
    var elementY = 0;
    var elementFontSize = efis.fontSize * 2;
    var elementSmallFontSize = elementFontSize / 2;
    var altitudeDisplay = parseInt(data.altitude / efis.AltitudeUnitFactor);

    // Draw translucid element space
    efis.efisContext.globalAlpha = 0.2;
    efis.efisContext.fillStyle = 'black';
    efis.efisContext.fillRect(elementX, elementY, elementWidth, elementHeight);
    efis.efisContext.globalAlpha = 1.0;

    // Draw Altitude Scroller
    var startAltitude = parseInt(altitudeDisplay - ((efis.AltitudeFOV / 2) + efis.AltitudeSteps));
    var endAltitude = parseInt(altitudeDisplay + ((efis.AltitudeFOV / 2) + efis.AltitudeSteps));

    var numberX = elementX;
    var lineX1 = numberX;
    var lineX2 = numberX + ((elementWidth / 8) * 0.9);
    var textX = numberX + (elementWidth / 8);

    for(i=startAltitude; i<=endAltitude;i++) {
        if(i % efis.AltitudeSteps == 0) {
            var startingPosY = (elementHeight / 2) - (altitudeDisplay * (elementHeight / efis.AltitudeFOV));
            var numberY = startingPosY + (i * (elementHeight / efis.AltitudeFOV) );
            numberY = elementHeight - numberY;

            efis.efisContext.beginPath();
            efis.efisContext.fillStyle = 'white';
            efis.efisContext.font = elementSmallFontSize + 'px ' + efis.DefaultFont;
            efis.efisContext.textAlign = "left";
            efis.efisContext.fillText(i, textX, numberY + (elementSmallFontSize / 2.5));
            efis.efisContext.moveTo(lineX1, numberY);
            efis.efisContext.lineTo(lineX2, numberY);
            efis.efisContext.moveTo(lineX1, numberY + ((elementHeight / efis.AltitudeFOV) * (efis.AltitudeSteps /2)));
            efis.efisContext.lineTo(lineX2, numberY + ((elementHeight / efis.AltitudeFOV) * (efis.AltitudeSteps /2)));
            efis.efisContext.strokeStyle = 'white';
            efis.efisContext.stroke();
            efis.efisContext.closePath();
        }
    }
    
    // Draw box on center of the element
    efis.efisContext.beginPath();
    drawBoxWithTextAbsPosition(efis.efisContext, altitudeDisplay, 'white', elementFontSize, 'right', P(elementX, (elementHeight / 2) - (elementFontSize / 2)), P(elementWidth, elementFontSize), '000', efis.AltitudeSmallTextNumber);

    // Draw unit
    efis.efisContext.fillStyle = 'white';
    efis.efisContext.font = elementSmallFontSize + 'px ' + efis.DefaultFont;
    efis.efisContext.textAlign = "right";
    efis.efisContext.fillText(efis.AltitudeUnitLabel, elementX + elementWidth, (elementHeight / 2) + elementFontSize);
    efis.efisContext.closePath();




}

function AngleToRadians(angle) {
    return angle * Math.PI / 180;
}

function RadiansToAngle(radians) {
    return radians * 180 / Math.PI;
}

function drawEfisBankAngle() {
    var elementWidth = (efis.efisWidth / 3);
    var elementHeight = (efis.efisHeight / 6);
    var elementX = (efis.efisWidth / 2) - (elementWidth / 2);
    var elementY = efis.blockHeight;
    var elementCenterX = elementX + (elementWidth / 2);

    var lineLength = (efis.efisHeight / 2) - elementY;

    // Draw lines

    efis.efisContext.strokeStyle = 'white';
    var bankAngle = parseInt(data.rollAngle);
    var arrowAngle = 10;

    for(i=0; i<=60; i+=15) {
        var x1 = (efis.efisWidth / 2);
        var y1 = (efis.efisHeight / 2);
        var x2 = x1 + lineLength * Math.sin(AngleToRadians(i - bankAngle));
        var y2 = y1 - lineLength * Math.cos(AngleToRadians(i - bankAngle));

        efis.efisContext.strokeStyle = 'white';
        if(i == 0)
        {
            efis.efisContext.lineWidth = 3 * window.devicePixelRatio;
            efis.efisContext.strokeStyle = 'yellow';
        }
        else if(i == 60 || i == 30)
            efis.efisContext.lineWidth = 2 * window.devicePixelRatio;
        else
            efis.efisContext.lineWidth = 1 * window.devicePixelRatio;

        drawShorterLine(efis.efisContext, P(x1, y1), P(x2, y2), (efis.efisHeight / 2) - (efis.blockHeight * 2), 0);

        efis.efisContext.strokeStyle = 'white';
        if(i > 0) {
            x2 = (x1 - (lineLength * Math.sin(AngleToRadians(i + bankAngle))));
            y2 = (y1 - (lineLength * Math.cos(AngleToRadians(i + bankAngle))));
            drawShorterLine(efis.efisContext, P(x1, y1), P(x2, y2), (efis.efisHeight / 2) - (efis.blockHeight * 2), 0);
        }
    }

    // Draw bank arrow
    if(bankAngle > -60 && bankAngle < 60) {
        var arrowAngle = 12;
        bankAngle = 0; // Make yellow arrow always point up

        x1 = elementCenterX + (lineLength - efis.blockHeight) * Math.sin(AngleToRadians(bankAngle));
        y1 = (efis.efisHeight / 2) - (lineLength - efis.blockHeight) * Math.cos(AngleToRadians(bankAngle));

        var x2a = x1 - (efis.blockWidth * Math.sin(AngleToRadians(-arrowAngle + bankAngle)));
        var y2a = y1 + (efis.blockHeight * Math.cos(AngleToRadians(-arrowAngle + bankAngle)));
        var x2b = x1 - (efis.blockWidth * Math.sin(AngleToRadians(arrowAngle + bankAngle)));
        var y2b = y1 + (efis.blockHeight * Math.cos(AngleToRadians(arrowAngle + bankAngle)));

        efis.efisContext.fillStyle = 'yellow';
        efis.efisContext.beginPath();
        efis.efisContext.moveTo(x1, y1);
        efis.efisContext.lineTo(x2a, y2a);
        efis.efisContext.lineTo(x2b, y2b);
        efis.efisContext.lineTo(x1, y1);
        efis.efisContext.fill();
        efis.efisContext.closePath();
    }
}

function drawEfisBatterySection() {
    var elementWidth = efis.blockWidth * 2;
    var elementHeight = efis.blockHeight * 2;
    var elementX = efis.blockWidth * 3.5;
    var elementY = (efis.efisHeight) - elementHeight - (efis.blockHeight * 1.5);
    var padding = 1.5;
    var elementFont = efis.fontSize + 'px ' + efis.DefaultFont;
    var elementSmallFont = efis.fontSize * 0.8 + 'px ' + efis.DefaultFont;

    // Draw fuel % bar
    var barColor = 'limegreen';
    if(data.fuelPercent < 35)
        barColor = 'yellow';
    if(data.fuelPercent < 15)
        barColor = 'red';
    
    var displayText = data.battCellVoltage.toFixed(2) + 'v'
    var barH = efis.blockHeight / 1.5;
    var barX = elementX + efis.blockWidth * 1.5;
    var barY = elementY;

    drawTextWithShadow(efis.efisContext, 'Batt', elementX, barY + (barH / 1.3), elementSmallFont, 'left', 'white', 'black', padding);
    drawProgressBar(efis.efisContext, displayText, barX, barY, elementWidth, barH, data.fuelPercent, barColor, elementSmallFont, padding);

    // Draw RSSI % bar
    barColor = 'limegreen';
    if(data.rssiPercent < 50)
        barColor = 'yellow';
    if(data.rssiPercent < 20)
        barColor = 'red';
        
    barX = elementX + efis.blockWidth * 1.5;
    barY = elementY + (efis.blockHeight * 1);

    displayText = data.rssiPercent.toFixed(0) + '%'
    drawTextWithShadow(efis.efisContext, 'RSSI', elementX, barY + (barH / 1.3), elementSmallFont, 'left', 'white', 'black', padding);
    drawProgressBar(efis.efisContext, displayText, barX, barY, elementWidth, barH, data.rssiPercent, barColor, elementSmallFont, padding);

    // Draw battery icon and Voltage


}

function renderEFIS(data) {
    var startTime = performance.now();

    UpdateViewPortSize();

    // Declare basic objects and resize canvas to the hud available size
    efis.efisContext = efis.efisCanvas.getContext("2d"),
    efis.efisWidth = efis.hudView.offsetWidth;
    efis.efisHeight = efis.hudView.offsetHeight;
    efis.efisCanvas.width = efis.efisWidth;
    efis.efisCanvas.height = efis.efisHeight;
    efis.efisContext.clearRect(0, 0, efis.efisWidth, efis.efisHeight);
    
    // Line length gets the diagonal size of the EFIS.
    efis.lineLength = Math.pow((efis.efisWidth * efis.efisWidth) + (efis.efisHeight * efis.efisHeight), 0.5);
    efis.pitchDegPixels = efis.efisHeight / efis.verticalFOV;
    efis.sizeFactorEfis = efis.efisWidth / efis.horizontalFOV;
    efis.blockWidth = efis.efisWidth / efis.qtyBlocks;
    efis.blockHeight = efis.efisHeight / efis.qtyBlocks;
    efis.fontSize = parseInt((efis.blockHeight / 4) * efis.fontSizeFactor);

    // Draw Sky, Ground and Horizon
    drawEfisHorizonGroundSky();
    
    // Draw Artificial horizon step lines
    drawEfisArtifitialHorizonStepLines();
    
    // Draw Crosshair
    drawEfisCrosshair();
    
    // Draw Heading indicator
    drawEfisHeadingIndicator();
    
    // Draw Ground Speed
    drawEfisGroundSpeed();
    
    // Draw Altitude
    drawEfisAltitude();

    // Draw Vertical Speed
    drawEfisVerticalSpeed();

    // Draw Bank angle
    drawEfisBankAngle();

    // Draw Battery information
    //drawEfisBatterySection();

    // Debug Info
    var endTime = performance.now();
    var elapsedTime = (endTime - startTime).toFixed(1);

    if(0==1) {
        var debugFont = (efis.fontSize / 1.5) + 'px ' + efis.DefaultFont
        var debugText = efis.efisWidth + 'x' + efis.efisHeight + 'px, ' + elapsedTime + 'ms';
        
        drawTextWithShadow(efis.efisContext, debugText, efis.blockWidth * 3, efis.blockHeight * 0.5, debugFont, 'left', 'white', 'black', 1);

    }
}