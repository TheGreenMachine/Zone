package com.team1816.season.auto.path;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class BottomToSideThree extends AutoPath {
    public BottomToSideThree(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(7.62, 1.2, Rotation2d.fromDegrees(147)),
                new Pose2d(5.24, 2.86, Rotation2d.fromDegrees(147))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(120)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
