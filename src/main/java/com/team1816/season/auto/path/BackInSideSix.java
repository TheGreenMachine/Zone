package com.team1816.season.auto.path;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class BackInSideSix extends AutoPath {
    public BackInSideSix(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(3.29,5.94, Rotation2d.fromDegrees(301)),
                new Pose2d(3.78,5.14, Rotation2d.fromDegrees(301))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(480),
                Rotation2d.fromDegrees(480)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
