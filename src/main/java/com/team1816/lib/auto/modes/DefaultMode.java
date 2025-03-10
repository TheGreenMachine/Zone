package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DefaultMode extends AutoMode{
    @Override
    protected void routine() throws AutoModeEndedException {

    }

    public Pose2d getInitialPose() {
        return new Pose2d(0,0, robotState.allianceColor == Color.BLUE ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
    }
}
