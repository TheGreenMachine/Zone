package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.AlgaeCatcher;

public class AlgaeCatcherCompleteAction implements AutoAction {
    private RobotState robotState;
    private AlgaeCatcher algaeCatcher;
    private AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE desiredIntakeState;
    private AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE desiredPivotState;

    public AlgaeCatcherCompleteAction(AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE desiredIntakeState, AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE desiredPositionState) {
        this.robotState = Injector.get(RobotState.class);
        this.algaeCatcher = Injector.get(AlgaeCatcher.class);
        this.desiredIntakeState = desiredIntakeState;
        this.desiredPivotState = desiredPositionState;
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
        return algaeCatcher.isAlgaeCatcherIntakeInRange() && algaeCatcher.isAlgaeCatcherPivotInRange();
    }

    @Override
    public void done() {

    }
}
