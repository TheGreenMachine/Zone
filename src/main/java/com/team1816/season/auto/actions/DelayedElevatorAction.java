package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.season.subsystems.Elevator;

public class DelayedElevatorAction extends SeriesAction {
    public DelayedElevatorAction(Elevator.ELEVATOR_STATE desiredState, double timeToWait) {
        super (
                new SeriesAction(
                        new WaitAction(timeToWait),
                        new ElevatorCompleteAction(desiredState)
                )
        );
    }
}
