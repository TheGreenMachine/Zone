package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.AlgaeCatcher;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.DeepCageHanger;

public class CoralArmAction implements AutoAction {
    private RobotState robotState;
    private CoralArm coralArm;
    private CoralArm.INTAKE_STATE desiredIntakeState;
    private CoralArm.PIVOT_STATE desiredPivotState;

    public CoralArmAction(CoralArm.INTAKE_STATE desiredIntakeState, CoralArm.PIVOT_STATE desiredPivotState) {
        this.robotState = Injector.get(RobotState.class);
        this.coralArm = Injector.get(CoralArm.class);
        this.desiredIntakeState = desiredIntakeState;
        this.desiredPivotState = desiredPivotState;
    }
    @Override
    public void start() {
        coralArm.setDesiredIntakeState(desiredIntakeState);
        coralArm.setDesiredPivotState(desiredPivotState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return  robotState.actualIntakeState == desiredIntakeState && robotState.actualPivotState == desiredPivotState;
    }

    @Override
    public void done() {

    }
}
