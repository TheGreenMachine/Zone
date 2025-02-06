package com.team1816.season.auto.modes;

import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.DynamicAutoScript2025;
import com.team1816.season.auto.actions.*;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;

public class DynamicPlace1 extends AutoMode {
    private ArrayList<DynamicAutoScript2025.REEF_LEVEL> coralPlacements;

    public DynamicPlace1(RobotState rs){
        super.trajectoryActions = rs.dynamicAutoScript2025.getAutoTrajectoryActionsIgnoreEmpty();

        coralPlacements = rs.dynamicAutoScript2025.getCoralPlacements();
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
                        new PlaceCoralSeriesAction(coralPlacements.get(0).getEquivalentElevatorState(), coralPlacements.get(0).getEquivalentCoralArmPivotState(), true)
                )
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return robotState.dynamicAutoScript2025.getStartPos();
    }
}
