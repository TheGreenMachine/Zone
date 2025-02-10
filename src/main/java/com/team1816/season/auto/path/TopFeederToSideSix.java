package com.team1816.season.auto.path;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class TopFeederToSideSix extends  AutoPath {
    public TopFeederToSideSix(Color color) {
        super(color);
    }


    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(1.19, 6.84, Rotation2d.fromDegrees(330)),
                new Pose2d(3.78,5.14, Rotation2d.fromDegrees(330))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(307),
                Rotation2d.fromDegrees(300)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
