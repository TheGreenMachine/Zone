package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.AlgaeCatcher;
import com.team1816.season.subsystems.CoralArm;

public class AlgaeCatcherAction implements AutoAction {
    private RobotState robotState;
    private AlgaeCatcher algaeCatcher;
    private AlgaeCatcher.ALGAE_CATCHER_STATE desiredCatcherState;
    private AlgaeCatcher.POSITION_STATE desiredPositionState;

    public AlgaeCatcherAction(AlgaeCatcher.ALGAE_CATCHER_STATE desiredCatcherState, AlgaeCatcher.POSITION_STATE desiredPositionState) {
        this.robotState = Injector.get(RobotState.class);
        this.algaeCatcher = Injector.get(AlgaeCatcher.class);
        this.desiredCatcherState = desiredCatcherState;
        this.desiredPositionState = desiredPositionState;
    }
    @Override
    public void start() {
        algaeCatcher.setDesiredState(desiredCatcherState);
        algaeCatcher.setDesiredPositionState(desiredPositionState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return algaeCatcher.isAlgaeCatcherPivotInRange() && algaeCatcher.isAlgaeCatcherIntakeInRange();
    }

    @Override
    public void done() {

    }
}
