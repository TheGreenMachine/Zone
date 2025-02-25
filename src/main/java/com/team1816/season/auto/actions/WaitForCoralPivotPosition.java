package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.CoralArm;

public class WaitForCoralPivotPosition implements AutoAction {
    private final RobotState robotState;
    private final CoralArm coralArm;
    private final CoralArm.PIVOT_STATE pivotState;

    public WaitForCoralPivotPosition(CoralArm.PIVOT_STATE pivotPosition){
        robotState = Injector.get(RobotState.class);
        coralArm = Injector.get(CoralArm.class);
        pivotState = pivotPosition;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return coralArm.isCoralArmPivotInRange() && robotState.actualCoralArmPivotState == pivotState;
    }

    @Override
    public void done() {

    }
}
