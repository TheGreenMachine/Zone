package com.team1816.season.auto.path;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class BackOutSideSix extends AutoPath {
    public BackOutSideSix(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(3.78,5.14, Rotation2d.fromDegrees(301-180)),
                new Pose2d(3.29,5.94, Rotation2d.fromDegrees(301-180))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(300),
                Rotation2d.fromDegrees(300)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
