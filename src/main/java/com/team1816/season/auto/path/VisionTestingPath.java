package com.team1816.season.auto.path;

import com.team1816.core.configuration.Constants;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class VisionTestingPath extends AutoPath {
    public VisionTestingPath(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(Constants.topStartPose.getTranslation(), Rotation2d.fromDegrees(240)),
                new Pose2d(Constants.reef1APose.getTranslation(), Rotation2d.fromDegrees(240)),
                new Pose2d(4.5, 7, Rotation2d.fromDegrees(180)),
                new Pose2d(3, 7, Rotation2d.fromDegrees(180)),
                new Pose2d(7, 7, Rotation2d.fromDegrees(0)),
                new Pose2d(3, 7, Rotation2d.fromDegrees(179)),
                new Pose2d(7, 7, Rotation2d.fromDegrees(0)),
                new Pose2d(Constants.reef1APose.getTranslation(), Rotation2d.fromDegrees(240))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Constants.topStartPose.getRotation(),
                Constants.reef1APose.getRotation(),
                Rotation2d.fromDegrees(270),
                Rotation2d.fromDegrees(270),
                Rotation2d.fromDegrees(270),
                Rotation2d.fromDegrees(270),
                Rotation2d.fromDegrees(270),
                Constants.reef1APose.getRotation()
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
