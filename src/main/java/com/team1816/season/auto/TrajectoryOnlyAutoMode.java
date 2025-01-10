package com.team1816.season.auto;

import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryOnlyAutoMode extends AutoMode {
    private RobotState rs;

    public TrajectoryOnlyAutoMode(RobotState rs){
        super(rs.dynamicAutoScript2025.getAutoTrajectoryActionsIgnoreEmpty());
        this.rs = rs;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        for (TrajectoryAction trajectoryAction : trajectoryActions) {
            runAction(trajectoryAction);
        }
    }

    @Override
    public Pose2d getInitialPose() {
        return robotState.dynamicAutoScript2025.getStartPos();
    }
}
