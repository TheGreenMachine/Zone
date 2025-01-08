package com.team1816.season.auto;

import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryOnlyAutoMode extends AutoMode {
    private RobotState rs;

    public TrajectoryOnlyAutoMode(RobotState rs){
        super(rs.dynamicAutoScript2025.getAutoTrajectoryActions());
        this.rs = rs;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        for(int i = 0; i < trajectoryActions.size(); i++)
            runAction(trajectoryActions.get(i));
    }
}
