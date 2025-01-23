package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.AlgaeArm;
import com.team1816.season.subsystems.Pneumatic;

public class PneumaticAction implements AutoAction {
    private RobotState robotState;
    private Pneumatic pneumatic;
    private AlgaeArm.ALGAE_ARM_STATE desiredState;

    public PneumaticAction(AlgaeArm.ALGAE_ARM_STATE desiredState) {
        this.robotState = Injector.get(RobotState.class);
        this.pneumatic = Injector.get(Pneumatic.class);
//        this.desiredState = ;
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
//        return robotState.actualPneumaticState == ;
    }

    @Override
    public void done() {

    }
}
