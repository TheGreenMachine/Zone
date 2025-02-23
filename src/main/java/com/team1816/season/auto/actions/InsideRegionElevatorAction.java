package com.team1816.season.auto.actions;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.actions.WaitUntilInsideRegion;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Translation2d;

public class InsideRegionElevatorAction extends SeriesAction {
    public InsideRegionElevatorAction(Elevator.ELEVATOR_STATE desiredState, Translation2d bottomLeft, Translation2d topRight, Color color) {
        super (
                new SeriesAction(
                        new WaitUntilInsideRegion(bottomLeft, topRight, "raiseElevatorRegion"),
                        new ElevatorCompleteAction(desiredState)
                ) // bottomLeft = 450, 0    topRight = 600, 800
        );
    }
}
