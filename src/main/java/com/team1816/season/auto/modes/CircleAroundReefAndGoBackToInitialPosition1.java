package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CircleAroundReefAndGoBackToInitialPosition1 extends AutoMode {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
                new PathPlannerAction("Circle Around Reef And Go Back To Initial Position 1")
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(new Translation2d(6.710, 6.950), new Rotation2d(-166.718));
    }
}
