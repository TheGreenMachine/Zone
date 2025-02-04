package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.CoralArm;

public class CoralArmIntakeAction implements AutoAction {
    private RobotState robotState;
    private CoralArm coralArm;
    private CoralArm.INTAKE_STATE desiredIntakeState;

    public CoralArmIntakeAction(CoralArm.INTAKE_STATE desiredIntakeState) {
        this.robotState = Injector.get(RobotState.class);
        this.coralArm = Injector.get(CoralArm.class);
        this.desiredIntakeState = desiredIntakeState;
    }
    @Override
    public void start() {
        coralArm.setDesiredIntakeState(desiredIntakeState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return  robotState.actualCoralArmIntakeState == desiredIntakeState;
    }

    @Override
    public void done() {

    }
}
