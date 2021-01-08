function buildBatteryPack() {
    var MaxAllowedCells = 200;


    var CellsInSeries = $('#CellsInSeries').val();
    var CellsInParallel = $('#CellsInParallel').val();
    var TotalCellCount = CellsInSeries * CellsInParallel;

    if (TotalCellCount > MaxAllowedCells) {
        alert('Too many cells. The maximum is ' + MaxAllowedCells + '.');
        return;
    }

    // Get all dividers
    var PackDividers = GetNumberDividers(TotalCellCount);

    // Start iterating the dividers
    for (var i = 0; i < PackDividers.length; i++) {
        var CurrentPackRows = PackDividers[i];
        var CurrentPackColumns = TotalCellCount / CurrentPackRows;

        // Get rows dividers so we can plot all posible layers
        var RowDividers = GetNumberDividers(CurrentPackColumns);

        for (var j = 0; j < RowDividers.length; j++) {
            CurrentPackColumns = TotalCellCount / CurrentPackRows;
            if ((CurrentPackColumns / RowDividers[j]) % CellsInParallel == 0 || RowDividers[j] == 1) {
                // Print pack layout
                var PackLayout = CellsInSeries + "S" + CellsInParallel + "P " + CurrentPackRows + "x" + (CurrentPackColumns / RowDividers[j]) + "x" + RowDividers[j]

                // Draw Pack
                var PackCode = BuildPack(CurrentPackRows, CellsInSeries, CellsInParallel, RowDividers[j]);
            }
        }
    }
}

function BuildPack(CurrentPackRows, CellsInSeries, CellsInParallel, QtyLayers) {
    var TotalCellCount = CellsInSeries * CellsInParallel;
    var CurrentPackColumns = TotalCellCount / CurrentPackRows;
    var LayerDivider = CurrentPackColumns / QtyLayers;

    var CurrentCellRow = 0;
    var CurrentCellColumn = 0;
    var MaxAllowedColumn = CurrentPackColumns -1;
    var CellAddindDirection = 1;
    var CellPole = false;
    var JustChangedDirection = false;
    var CellParallelNumber = 1;
    var LayerCellCounter = 1;
    var LayerNumber = 1;

    // Create array with empty cells
    var arrCells = [];
    for (var i = 0; i < CurrentPackRows; i++) {
        var cellLine = new Array(CurrentPackColumns);
        arrCells.push(cellLine);
    }

    for (var CellNumber = 1; CellNumber <= TotalCellCount; CellNumber++) {
        var objCell = {
            cellNumber: CellNumber,
            cellPole: CellPole,
            cellLayer: LayerNumber,
        };
        arrCells[CurrentCellRow][CurrentCellColumn] = objCell;

        CellParallelNumber++;

        if (CellParallelNumber > CellsInParallel) {
            CellParallelNumber = 1;
            CellPole = !CellPole;
        }

        // Walk to the next cell
        if (CellNumber < TotalCellCount) {
            // Check if the row ended
            if ((CurrentCellColumn + 1) > MaxAllowedColumn && CellAddindDirection == 1) {
                // Reached the end column of the pack
                CellAddindDirection = -1;
                CurrentCellRow++;
                JustChangedDirection = true;
                LayerCellCounter = 0;
            } else if ((CurrentCellColumn - 1) < 0 && CellAddindDirection == -1) {
                // Reached the end column of the pack
                CellAddindDirection = 1;
                CurrentCellRow++;
                JustChangedDirection = true;
                LayerCellCounter = 0;
            } else {
                // Just walk
                CurrentCellColumn += CellAddindDirection;
                JustChangedDirection = false;
            }

            // If the layer ended, start new layer
            if(LayerCellCounter == LayerDivider && !JustChangedDirection) {
                LayerCellCounter=0;
                LayerNumber+=CellAddindDirection;
            }
        }

        LayerCellCounter++;
    }

    console.log(arrCells);
}



function GetNumberDividers(inputNumber) {
    var objReturn = [];

    for (var i = 1; i < inputNumber; i++)
        if (inputNumber % i == 0)
            objReturn.push(i);

    return objReturn;
}









$('#btnbuildpack').click(function () {
    buildBatteryPack();
});