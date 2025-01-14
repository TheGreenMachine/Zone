package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.AlgaeArm;

public class AlgaeArmAction implements AutoAction {
    private RobotState robotState;
    private AlgaeArm algaeArm;
    private AlgaeArm.ALGAE_ARM_STATE desiredState;

    public AlgaeArmAction(AlgaeArm.ALGAE_ARM_STATE desiredState) {
        this.robotState = Injector.get(RobotState.class);
        this.algaeArm = Injector.get(AlgaeArm.class);
        this.desiredState = desiredState;
    }

    @Override
    public void start() {
        algaeArm.setDesiredState(desiredState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return robotState.actualAlgaeArmState == desiredState;
    }

    @Override
    public void done() {

    }
}
