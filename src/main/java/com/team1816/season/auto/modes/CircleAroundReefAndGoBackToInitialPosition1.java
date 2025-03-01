package com.team1816.season.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
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
        Pose2d origin = action.getPath().getPathPoses().get(0);
        if (AutoBuilder.shouldFlip()) {
            origin = FlippingUtil.flipFieldPose(origin);
        }

        return origin;
    }
}
