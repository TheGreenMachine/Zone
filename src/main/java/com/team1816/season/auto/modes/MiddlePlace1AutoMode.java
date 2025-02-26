package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.CoralArmQuickAction;
import com.team1816.season.auto.actions.DelayedElevatorAction;
import com.team1816.season.auto.actions.ElevatorQuickAction;
import com.team1816.season.auto.actions.PlaceCoralSeriesAction;
import com.team1816.season.auto.path.MiddleToSideThree;
import com.team1816.season.auto.path.MiddleToSideTwo;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public class MiddlePlace1AutoMode extends AutoMode {

    public MiddlePlace1AutoMode(Color color) {
        super(
                List.of(
                        new TrajectoryAction(
                                new MiddleToSideTwo(color)
                        )
                )
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
                new SeriesAction(
                        new ParallelAction(
                                new CoralArmQuickAction(CoralArm.INTAKE_STATE.INTAKE, CoralArm.PIVOT_STATE.FEEDER),
                                new ElevatorQuickAction(Elevator.ELEVATOR_STATE.FEEDER)
                        ),
                        new ParallelAction(
                                new DelayedElevatorAction(Elevator.ELEVATOR_STATE.L4, 1),
                                trajectoryActions.get(0)
                        ),
                        new PlaceCoralSeriesAction(Elevator.ELEVATOR_STATE.L4, CoralArm.PIVOT_STATE.L4, true)
                )
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return trajectoryActions.get(0).getTrajectory().getInitialPose();
    }
}
