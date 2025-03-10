package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.FieldPlacement;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.path.DriveOffLineMiddlePath;
import com.team1816.season.auto.path.DriveOffLineTopPath;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public class DriveOffLineTopAutoMode extends AutoMode {

    public DriveOffLineTopAutoMode(Color color, FieldPlacement fieldPlacement) {
        super(
                List.of(
                        new TrajectoryAction(
                                new DriveOffLineTopPath(color, fieldPlacement)
                        )
                )
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
                new SeriesAction(
                        trajectoryActions.get(0)
                )
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return trajectoryActions.get(0).getTrajectory().getInitialPose();
    }
}
