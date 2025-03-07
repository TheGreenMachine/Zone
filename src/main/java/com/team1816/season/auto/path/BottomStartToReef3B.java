package com.team1816.season.auto.path;

import com.team1816.core.configuration.Constants;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class BottomStartToReef3B extends AutoPath {
    public BottomStartToReef3B(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(Constants.bottomStartPose.getTranslation(), Rotation2d.fromDegrees(90)),
                new Pose2d(Constants.reef3BPose.getTranslation(), Rotation2d.fromDegrees(-231))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Constants.bottomStartPose.getRotation(),
                Constants.reef3BPose.getRotation()
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
