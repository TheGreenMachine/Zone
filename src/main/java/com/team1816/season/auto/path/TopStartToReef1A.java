package com.team1816.season.auto.path;

import com.team1816.core.configuration.Constants;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class TopStartToReef1A extends AutoPath {
    public TopStartToReef1A(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(Constants.topStartPose.getTranslation(), Rotation2d.fromDegrees(270)),
                new Pose2d(7.12, 6.45, Rotation2d.fromDegrees(270)),
                new Pose2d(Constants.reef1APose.getTranslation(), Rotation2d.fromDegrees(-129))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Constants.topStartPose.getRotation(),
                Rotation2d.fromDegrees(270),
                Constants.reef1APose.getRotation()
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
