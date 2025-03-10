package com.team1816.season.auto.path;

import com.team1816.core.configuration.Constants;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class MiddleStartToReef2A extends AutoPath {
    public MiddleStartToReef2A(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(Constants.middleStartPose.getTranslation(), Rotation2d.fromDegrees(180)),
                new Pose2d(Constants.reef2APose.getTranslation(), Rotation2d.fromDegrees(180))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Constants.middleStartPose.getRotation(),
                Constants.reef2APose.getRotation()
        );
    }   

    @Override
    protected boolean usingApp() {
        return true;
    }
}
