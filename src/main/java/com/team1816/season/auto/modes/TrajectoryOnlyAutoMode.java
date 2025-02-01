package com.team1816.season.auto.modes;

import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;

public class TrajectoryOnlyAutoMode extends AutoMode {
    private ArrayList<TrajectoryAction> actions = new ArrayList<>();

    public TrajectoryOnlyAutoMode(RobotState rs){
        actions.addAll(rs.dynamicAutoScript2025.getAutoTrajectoryActionsIgnoreEmpty());
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        for(TrajectoryAction action : actions)
            runAction(
                    action
            );
    }

    @Override
    public Pose2d getInitialPose() {
        return robotState.dynamicAutoScript2025.getStartPos();
    }
}
