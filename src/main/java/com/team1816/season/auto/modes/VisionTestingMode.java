package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.path.DriveOffLineBottomPath;
import com.team1816.season.auto.path.VisionTestingPath;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public class VisionTestingMode extends AutoMode {

    public VisionTestingMode(Color color) {
        super(
                List.of(
                        new TrajectoryAction(
                                new VisionTestingPath(color)
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
}
