package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.AlgaeCatcherAction;
import com.team1816.season.auto.actions.CoralArmAction;
import com.team1816.season.auto.actions.ElevatorAction;
import com.team1816.season.auto.actions.PlaceCoralSeriesAction;
import com.team1816.season.auto.path.*;
import com.team1816.season.subsystems.AlgaeCatcher;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class BottomPlace2Automode extends AutoMode {

    public BottomPlace2Automode() {
        super(
                List.of(
                        new TrajectoryAction(
                                new BottomToSideThree(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new SideThreeToCloseFeeder(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new CloseFeederToSideFour(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new SideFourOut(robotState.allianceColor)
                        ), new TrajectoryAction(
                                new SideFourIn(robotState.allianceColor)
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
                                new RotateSwerveAction(Rotation2d.fromDegrees(240)),
                                new AlgaeCatcherAction(AlgaeCatcher.ALGAE_CATCHER_STATE.OUTTAKE, AlgaeCatcher.POSITION_STATE.ALGAEL2)
                        ),
                        trajectoryActions.get(4)
                ));
    }

    @Override
    public Pose2d getInitialPose() {
        return trajectoryActions.get(0).getTrajectory().getInitialPose();
    }
}
