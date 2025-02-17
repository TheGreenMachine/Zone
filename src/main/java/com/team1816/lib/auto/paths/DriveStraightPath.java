package com.team1816.lib.auto.paths;

import com.team1816.core.configuration.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.List;

import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxVelMeters;

public class DriveStraightPath extends AutoPath {
    public DriveStraightPath() {
    }

    @Override
    public List<Pose2d> getWaypoints() {
        var waypoints = List.of(
                new Pose2d(0.0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(10, 0, Rotation2d.fromDegrees(0))
        );
        return waypoints;
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return null;
    }

    @Override
    protected List<Rotation2d> getReflectedWaypointHeadings() {
        return null;
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}