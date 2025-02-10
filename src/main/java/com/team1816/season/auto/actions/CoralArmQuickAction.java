package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.CoralArm;

public class CoralArmQuickAction implements AutoAction {
    private RobotState robotState;
    private CoralArm coralArm;
    private CoralArm.INTAKE_STATE desiredIntakeState;
    private CoralArm.PIVOT_STATE desiredPivotState;

    public CoralArmQuickAction(CoralArm.INTAKE_STATE desiredIntakeState, CoralArm.PIVOT_STATE desiredPivotState) {
        this.robotState = Injector.get(RobotState.class);
        this.coralArm = Injector.get(CoralArm.class);
        this.desiredIntakeState = desiredIntakeState;
        this.desiredPivotState = desiredPivotState;
    }
    @Override
    public void start() {
        coralArm.setDesiredState(desiredPivotState, desiredIntakeState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return robotState.actualCoralArmIntakeState == desiredIntakeState && robotState.actualCoralArmPivotState == desiredPivotState;    }

    @Override
    public void done() {

    }
}
