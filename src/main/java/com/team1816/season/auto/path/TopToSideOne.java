package com.team1816.season.auto.path;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class TopToSideOne extends AutoPath {
    public TopToSideOne(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(7.12, 7.45, Rotation2d.fromDegrees(270)),
                new Pose2d(6.5, 6.44, Rotation2d.fromDegrees(220)),
                new Pose2d(5.3, 5.2, Rotation2d.fromDegrees(-129))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(270),
                Rotation2d.fromDegrees(220),
                Rotation2d.fromDegrees(235)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
