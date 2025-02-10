package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.AlgaeCatcher;

public class AlgaeCatcherQuickAction implements AutoAction {
    private RobotState robotState;
    private AlgaeCatcher algaeCatcher;
    private AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE desiredIntakeState;
    private AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE desiredPivotState;

    public AlgaeCatcherQuickAction(AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE desiredIntakeState, AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE desiredPivotState) {
        this.robotState = Injector.get(RobotState.class);
        this.algaeCatcher = Injector.get(AlgaeCatcher.class);
        this.desiredIntakeState = desiredIntakeState;
        this.desiredPivotState = desiredPivotState;
    }
    @Override
    public void start() {
        algaeCatcher.setDesiredState(desiredIntakeState, desiredPivotState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        System.out.println(robotState.actualAlgaeCatcherIntakeState.name() + desiredIntakeState.name() + robotState.actualAlgaeCatcherPivotState.name() + desiredPivotState.name());

        return robotState.actualAlgaeCatcherIntakeState == desiredIntakeState && robotState.actualAlgaeCatcherPivotState == desiredPivotState;
    }

    @Override
    public void done() {

    }
}
