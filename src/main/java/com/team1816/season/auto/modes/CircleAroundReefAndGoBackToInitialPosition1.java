package com.team1816.season.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
        Pose2d origin = action.getPathInitialPose();
        if (AutoBuilder.shouldFlip()) {
            origin = FlippingUtil.flipFieldPose(origin);
        }

        Rotation2d angle = origin.getRotation().plus(Rotation2d.k180deg); // I am going to die
        origin = new Pose2d(origin.getTranslation(), angle);

        return origin;
    }
}
