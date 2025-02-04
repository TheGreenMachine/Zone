package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Elevator;

public class ElevatorAction implements AutoAction {
    private RobotState robotState;
    private Elevator elevator;
    private Elevator.ELEVATOR_STATE desiredState;

    public ElevatorAction(Elevator.ELEVATOR_STATE desiredState) {
        this.robotState = Injector.get(RobotState.class);
        this.elevator = Injector.get(Elevator.class);
        this.desiredState = desiredState;
    }

    @Override
    public void start() {
        elevator.setDesiredState(desiredState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return robotState.actualElevatorState == desiredState;
    }

    @Override
    public void done() {

    }
}
