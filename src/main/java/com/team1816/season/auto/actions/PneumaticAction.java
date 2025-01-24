package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.AlgaeArm;
import com.team1816.season.subsystems.Pneumatic;

public class PneumaticAction implements AutoAction {
    private final Pneumatic.PNEUMATIC_STATE needsPneumatic;
    private RobotState robotState;
    private Pneumatic pneumatic;

    public PneumaticAction(Pneumatic.PNEUMATIC_STATE needsPneumatic) {
        this.robotState = Injector.get(RobotState.class);
        this.pneumatic = Injector.get(Pneumatic.class);
        this.needsPneumatic = needsPneumatic;
    }

    @Override
    public void start() {
        pneumatic.setPneumatic();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return robotState.actualPneumaticState == needsPneumatic;
    }

    @Override
    public void done() {

    }
}
