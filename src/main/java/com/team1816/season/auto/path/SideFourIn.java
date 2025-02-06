package com.team1816.season.auto.path;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class SideFourIn extends AutoPath {
    public SideFourIn(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(3.32, 2.21, Rotation2d.fromDegrees(-303)),
                new Pose2d(3.74, 2.87, Rotation2d.fromDegrees(-303))

        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(240),
                Rotation2d.fromDegrees(240)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
