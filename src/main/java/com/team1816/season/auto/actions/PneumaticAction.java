package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Pneumatic;

public class PneumaticAction implements AutoAction {
    private RobotState robotState;
    private Pneumatic pneumatic;
    private Pneumatic.PNEUMATIC_STATE desiredState;

    public PneumaticAction() {
        this.robotState = Injector.get(RobotState.class);
        this.pneumatic = Injector.get(Pneumatic.class);
    }

    @Override
    public void start() {
        pneumatic.toggle();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return robotState.actualPneumaticState == pneumatic.getDesiredState();
    }

    @Override
    public void done() {

    }
}
