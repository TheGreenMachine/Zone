package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.core.states.RobotState;
import com.team1816.season.subsystems.AlgaeCatcher;

public class AlgaeCatcherAction implements AutoAction {
    private RobotState robotState;
    private AlgaeCatcher algaeCatcher;
    private AlgaeCatcher.ALGAE_CATCHER_STATE desiredState;

    @Override
    public void start() {
        this.robotState = Injector.get(RobotState.class);
        this.algaeCatcher = Injector.get(AlgaeCatcher.class);
        this.desiredState = desiredState;
    }

    @Override
    public void update() {
        algaeCatcher.setDesiredState(desiredState);
    }

    @Override
    public boolean isFinished() {
        return robotState.actualAlgaeCatcherState == desiredState;
    }

    @Override
    public void done() {

    }
}
