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
                new Pose2d(7.12, 0.53, Rotation2d.fromDegrees(90)),
                new Pose2d(6.41, 1.70, Rotation2d.fromDegrees(130)),
                new Pose2d(5.24, 2.86, Rotation2d.fromDegrees(129))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(90),
                Rotation2d.fromDegrees(130),
                Rotation2d.fromDegrees(120)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
