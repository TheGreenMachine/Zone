package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.path.MiddleStartToReef2A;
import com.team1816.season.auto.path.Reef2AToBottomFeeder;
import com.team1816.season.auto.path.Reef2AToTopFeeder;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class MiddlePlace1AutoMode extends AutoMode {
    private final String endingFeeder;

    public MiddlePlace1AutoMode(Color color, String endingFeeder) {
        super(
                List.of(
                        new TrajectoryAction(
                                new MiddleStartToReef2A(color)
                        ),
                        new TrajectoryAction(
                                new Reef2AToTopFeeder(color)
                        ),
                        new TrajectoryAction(
                                new Reef2AToBottomFeeder(color)
                        )
                )
        );
        this.endingFeeder = endingFeeder;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
                new SeriesAction(
                        new ParallelAction(
                                new DelayedElevatorAction(Elevator.ELEVATOR_STATE.L4, 1),
                                trajectoryActions.get(0)
                        ),
                        new WaitForCoralPivotPosition(CoralArm.PIVOT_STATE.L4),
                        new WaitAction(0.5),
                        new OuttakeCoralSeriesAction(),
                        endingFeeder.equals("top") ? trajectoryActions.get(1) : endingFeeder.equals("bottom") ? trajectoryActions.get(2) : new WaitAction(0)
                )
        );
    }
}
