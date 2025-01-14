package com.team1816.core.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Elevator;

public class ElevatorAction implements AutoAction {
    private RobotState robotState;
    private Elevator shooter;
    private Elevator.ELEVATOR_STATE desiredElevatorState;

    public ElevatorAction(Elevator.ELEVATOR_STATE desiredElevatorState) {
        this.robotState = Injector.get(RobotState.class);
        this.shooter = Injector.get(Elevator.class);
        this.desiredElevatorState = desiredElevatorState;
    }


    @Override
    public void start() {
        shooter.setDesiredState(desiredElevatorState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return robotState.actualElevatorState == desiredElevatorState;
    }

    @Override
    public void done() {

    }
}
