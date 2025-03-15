package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Ramp;

public class RampQuickAction implements AutoAction {
    private RobotState robotState;
    private Ramp ramp;
    private Ramp.RAMP_STATE desiredRampState;

    public RampQuickAction(Ramp.RAMP_STATE desiredRampState) {
        this.robotState = Injector.get(RobotState.class);
        this.ramp = Injector.get(Ramp.class);
        this.desiredRampState = desiredRampState;
    }
    @Override
    public void start() {
        ramp.setDesiredState(desiredRampState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return robotState.actualRampState == desiredRampState;
    }

    @Override
    public void done() {

    }
}
