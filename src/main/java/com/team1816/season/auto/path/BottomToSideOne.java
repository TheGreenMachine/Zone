package com.team1816.season.auto.path;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class BottomToSideOne extends AutoPath {
    public BottomToSideOne(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(7.1, 0.96, Rotation2d.fromDegrees(120)),
                new Pose2d(5.87, 4.93, Rotation2d.fromDegrees(140)),
                new Pose2d(5.14, 5.47, Rotation2d.fromDegrees(140))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(240)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
