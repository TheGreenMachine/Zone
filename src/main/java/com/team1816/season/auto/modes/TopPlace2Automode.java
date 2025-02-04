package com.team1816.season.auto.modes;

import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.path.*;
import com.team1816.season.subsystems.AlgaeCatcher;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

public class TopPlace2Automode extends AutoMode {

    public TopPlace2Automode() {
        super(
                List.of(
                        new TrajectoryAction(
                                new FarProcessorToSideOne(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new SideOneToFarFeeder(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new FarFeederToSideSix(robotState.allianceColor)
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
                                new CoralArmAction(CoralArm.INTAKE_STATE.INTAKE, CoralArm.PIVOT_STATE.REST),
                                new ElevatorAction(Elevator.ELEVATOR_STATE.REST)
                        ),
                        trajectoryActions.get(0),
                        new ParallelAction(
                                new ElevatorAction(Elevator.ELEVATOR_STATE.L4),
                                new CoralArmAction(CoralArm.INTAKE_STATE.HOLD, CoralArm.PIVOT_STATE.L4)
                        ),
                        new PlaceCoralSeriesAction(),
                        new ParallelAction(
                                new CoralArmAction(CoralArm.INTAKE_STATE.REST, CoralArm.PIVOT_STATE.REST),
                                new ElevatorAction(Elevator.ELEVATOR_STATE.REST)
                        ),
                        trajectoryActions.get(1),
                        new ParallelAction(
                                new ElevatorAction(Elevator.ELEVATOR_STATE.FEEDER),
                                new CoralArmAction(CoralArm.INTAKE_STATE.REST, CoralArm.PIVOT_STATE.FEEDER)
                        ),
                        new CoralArmAction(CoralArm.INTAKE_STATE.INTAKE, CoralArm.PIVOT_STATE.FEEDER),
                        new ParallelAction(
                                new CoralArmAction(CoralArm.INTAKE_STATE.HOLD, CoralArm.PIVOT_STATE.REST),
                                new ElevatorAction(Elevator.ELEVATOR_STATE.REST)
                        ),
                        trajectoryActions.get(2),
                        new ParallelAction(
                                new ElevatorAction(Elevator.ELEVATOR_STATE.L1),
                                new CoralArmAction(CoralArm.INTAKE_STATE.HOLD, CoralArm.PIVOT_STATE.L1)
                        ),
                        new PlaceCoralSeriesAction(),
                        trajectoryActions.get(3),
                        new ParallelAction(
                                new CoralArmAction(CoralArm.INTAKE_STATE.REST, CoralArm.PIVOT_STATE.REST),
                                new ElevatorAction(Elevator.ELEVATOR_STATE.REST),
                                new RotateSwerveAction(Rotation2d.fromDegrees(480)),
                                new AlgaeCatcherAction(AlgaeCatcher.ALGAE_CATCHER_STATE.OUTTAKE, AlgaeCatcher.POSITION_STATE.ALGAEL2),
                                new AlgaeCatcherAction(AlgaeCatcher.ALGAE_CATCHER_STATE.STOP, AlgaeCatcher.POSITION_STATE.STOW)
                        ),
                        trajectoryActions.get(4)


                        ));
    }

    @Override
    public Pose2d getInitialPose() {
        return trajectoryActions.get(0).getTrajectory().getInitialPose();
    }
}
