package com.team1816.season.auto.path;

import com.team1816.core.configuration.Constants;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class Reef2AToTopFeeder extends AutoPath {
    public Reef2AToTopFeeder(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(Constants.reef2APose.getTranslation(), Rotation2d.fromDegrees(90)),
                new Pose2d(5.35,6.34, Rotation2d.fromDegrees(145)),
                new Pose2d(Constants.topFeederPose.getTranslation(), Rotation2d.fromDegrees(180))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Constants.reef2APose.getRotation(),
                Rotation2d.fromDegrees(180),
                Constants.topFeederPose.getRotation()
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
