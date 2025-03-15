package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.path.BottomStartToReef3B;
import com.team1816.season.auto.path.Reef3BToBottomFeeder;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class OuttakeL234Mode extends AutoMode {

    public OuttakeL234Mode() {}

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
                new SeriesAction(
                        new CoralArmQuickIntakeAction(CoralArm.INTAKE_STATE.OUTTAKE),
                        new WaitForCoralL234BeamBreakUntriggeredAction(),
                        new WaitAction(2),
                        new SetFeederStatesAction(false)
                )
        );
    }
}
