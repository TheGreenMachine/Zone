package com.team1816.season.auto.path;

import com.team1816.core.configuration.Constants;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class Reef2AToBottomFeeder extends AutoPath {
    public Reef2AToBottomFeeder(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(Constants.reef2APose.getTranslation(), Rotation2d.fromDegrees(270)),
                new Pose2d(5.35,1.5, Rotation2d.fromDegrees(215)),
                new Pose2d(Constants.bottomFeederPose.getTranslation(), Rotation2d.fromDegrees(180))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Constants.reef2APose.getRotation(),
                Rotation2d.fromDegrees(180),
                Constants.bottomFeederPose.getRotation()
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
