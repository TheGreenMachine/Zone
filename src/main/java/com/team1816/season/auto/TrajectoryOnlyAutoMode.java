package com.team1816.season.auto;

import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.modes.AutoMode;

public class TrajectoryOnlyAutoMode extends AutoMode {
    private RobotState rs;

    public TrajectoryOnlyAutoMode(RobotState rs){
        this.rs = rs;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
    }
}
