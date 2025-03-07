package com.team1816.season.auto.path;

import com.team1816.core.configuration.Constants;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class Reef1AToTopFeeder extends AutoPath {
    public Reef1AToTopFeeder(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(Constants.reef1APose.getTranslation(), Rotation2d.fromDegrees(145)),
                new Pose2d(Constants.topFeederPose.getTranslation(), Rotation2d.fromDegrees(145))
                );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Constants.reef1APose.getRotation(),
                Constants.topFeederPose.getRotation()
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
