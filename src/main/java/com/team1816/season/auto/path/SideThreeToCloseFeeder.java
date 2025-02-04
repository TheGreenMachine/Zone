package com.team1816.season.auto.path;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class SideThreeToCloseFeeder extends AutoPath {
    public SideThreeToCloseFeeder(Color color) {
        super(color);
    }
    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(5.24, 2.86, Rotation2d.fromDegrees(-135)),
                new Pose2d(1.25, 1, Rotation2d.fromDegrees(-175))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
                Rotation2d.fromDegrees(230),
                Rotation2d.fromDegrees(230)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
