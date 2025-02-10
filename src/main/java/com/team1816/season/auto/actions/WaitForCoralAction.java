package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;

public class WaitForCoralAction implements AutoAction {
    private RobotState robotState;

    public WaitForCoralAction(){
        robotState = Injector.get(RobotState.class);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return robotState.isCoralBeamBreakTriggered;
    }

    @Override
    public void done() {

    }
}
