package com.team1816.season.auto.path;

import com.team1816.core.configuration.Constants;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class Reef3BToBottomFeeder extends AutoPath {
    public Reef3BToBottomFeeder(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(Constants.reef3BPose.getTranslation(), Rotation2d.fromDegrees(-135)),
                new Pose2d(Constants.bottomFeederPose.getTranslation(), Rotation2d.fromDegrees(-175))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Constants.reef3BPose.getRotation(),
                Constants.bottomFeederPose.getRotation()
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
