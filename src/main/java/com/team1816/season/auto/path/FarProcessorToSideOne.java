package com.team1816.season.auto.path;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class FarProcessorToSideOne extends AutoPath {
    public FarProcessorToSideOne(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                        new Pose2d(7.13, 7.51, Rotation2d.fromDegrees(-131)),
                        new Pose2d(5.02, 5.21, Rotation2d.fromDegrees(-131))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(240)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
