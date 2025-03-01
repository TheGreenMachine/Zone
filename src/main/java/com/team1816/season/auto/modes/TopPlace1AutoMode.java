package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.path.Reef1AToTopFeeder;
import com.team1816.season.auto.path.TopStartToReef1A;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class TopPlace1AutoMode extends AutoMode {
    private final boolean endAtFeeder;

    public TopPlace1AutoMode(Color color, boolean endAtFeeder){
        super(
                List.of(
                        new TrajectoryAction(
                                new TopStartToReef1A(robotState.allianceColor)
                        ),
                        new TrajectoryAction(
                                new Reef1AToTopFeeder(robotState.allianceColor)
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
                        new WaitAction(6),
                        new OuttakeCoralSeriesAction(),
                        endAtFeeder ? trajectoryActions.get(1) : new WaitAction(0)
                )
        );
    }
}
