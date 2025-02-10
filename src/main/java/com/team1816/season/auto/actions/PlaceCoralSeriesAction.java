package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;

public class PlaceCoralSeriesAction extends SeriesAction {
    public PlaceCoralSeriesAction(Elevator.ELEVATOR_STATE desiredElevatorState, CoralArm.PIVOT_STATE desiredCoralArmPivot, boolean resetElevatorAndCoralArmAfter) {
        super(
                resetElevatorAndCoralArmAfter ?
                        new SeriesAction(
                                new ParallelAction(
                                        new ElevatorCompleteAction(desiredElevatorState),
                                        new CoralArmCompleteAction(CoralArm.INTAKE_STATE.HOLD, desiredCoralArmPivot)
                                ),
                                new OuttakeCoralSeriesAction(),
                                new ParallelAction(
                                        new CoralArmQuickAction(CoralArm.INTAKE_STATE.INTAKE, CoralArm.PIVOT_STATE.FEEDER),
                                        new ElevatorQuickAction(Elevator.ELEVATOR_STATE.FEEDER)
                                )
                        )
                :
                        new SeriesAction(
                                new ParallelAction(
                                        new ElevatorCompleteAction(desiredElevatorState),
                                        new CoralArmCompleteAction(CoralArm.INTAKE_STATE.HOLD, desiredCoralArmPivot)
                                ),
                                new OuttakeCoralSeriesAction()
                        )
        );
    }
}