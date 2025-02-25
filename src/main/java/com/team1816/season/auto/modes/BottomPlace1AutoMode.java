package com.team1816.season.auto.modes;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.path.BottomToSideThree;
import com.team1816.season.auto.path.SideThreeToCloseFeeder;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

public class BottomPlace1AutoMode extends AutoMode {
    private boolean endAtFeeder;

    public BottomPlace1AutoMode(Color color, boolean endAtFeeder) {
        super(
                List.of(
                        new TrajectoryAction(
                                new BottomToSideThree(color)
                        ),
                        new TrajectoryAction(
                                new SideThreeToCloseFeeder(color)
                        )
                )
        );
        this.endAtFeeder = endAtFeeder;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
                new SeriesAction(
                        new ParallelAction(
                                new DelayedElevatorAction(Elevator.ELEVATOR_STATE.L4, 2),
                                trajectoryActions.get(0)
                        ),
                        new WaitForCoralPivotPosition(CoralArm.PIVOT_STATE.L4),
                        new WaitAction(0.5),
                        new OuttakeCoralSeriesAction(),
                        endAtFeeder ? trajectoryActions.get(1) : new WaitAction(0)
                )
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return trajectoryActions.get(0).getTrajectory().getInitialPose();
    }
}
