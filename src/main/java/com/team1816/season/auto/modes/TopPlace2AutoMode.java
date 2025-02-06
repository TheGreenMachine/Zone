package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.path.*;
import com.team1816.season.subsystems.AlgaeCatcher;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class TopPlace2AutoMode extends AutoMode {
    public TopPlace2AutoMode(Color color){
        super(
                List.of(
                        new TrajectoryAction(
                                new TopToSideOne(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new SideOneToTopFeeder(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new TopFeederToSideSix(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new BackOutSideSix(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new BackInSideSix(robotState.allianceColor)
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
                        trajectoryActions.get(0),
                        new PlaceCoralSeriesAction(Elevator.ELEVATOR_STATE.L4, CoralArm.PIVOT_STATE.L4, true),
                        trajectoryActions.get(1),
                        new WaitForCoralAction(),
                        trajectoryActions.get(2),
                        new PlaceCoralSeriesAction(Elevator.ELEVATOR_STATE.L4, CoralArm.PIVOT_STATE.L4, true),
                        trajectoryActions.get(3),
                        new ParallelAction(
                                new RotateSwerveAction(Rotation2d.fromDegrees(robotState.allianceColor == Color.BLUE ? 120 : 300)),
                                new AlgaeCatcherQuickAction(AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE.OUTTAKE, AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE.ALGAE1)
                        ),
                        trajectoryActions.get(4)
                ));
    }
    @Override
    public Pose2d getInitialPose() {
        return trajectoryActions.get(0).getTrajectory().getInitialPose();
    }
}
