package com.team1816.season.auto.actions;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.DeepCageHanger;
import com.team1816.season.subsystems.Elevator;

public class DeepCageHangerAction implements AutoAction {
    private RobotState robotState;
    private DeepCageHanger deepCageHanger;
    private DeepCageHanger.DEEPCAGEHANGER_STATE desiredState;

    public DeepCageHangerAction(DeepCageHanger.DEEPCAGEHANGER_STATE desiredState) {
        this.robotState = Injector.get(RobotState.class);
        this.deepCageHanger = Injector.get(DeepCageHanger.class);
        this.desiredState = desiredState;
    }

    @Override
    public void start() {
        deepCageHanger.setDesiredState(desiredState);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return robotState.actualDeepCageHangerState == desiredState;
    }

    @Override
    public void done() {

    }
}