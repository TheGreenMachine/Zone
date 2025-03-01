package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;

import edu.wpi.first.math.geometry.Pose2d;

public class CircleAroundReefAndGoBackToInitialPosition1 extends AutoMode {
    private final PathPlannerAction action = new PathPlannerAction("Circle Around Reef And Go Back To Initial Position 1");

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
                action
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return action.getPath().flipPath().getPathPoses().get(0);
    }
}
