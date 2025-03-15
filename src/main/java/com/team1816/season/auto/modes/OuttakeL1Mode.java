package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.RampQuickAction;
import com.team1816.season.auto.actions.SetFeederStatesAction;
import com.team1816.season.auto.actions.WaitForCoralL234BeamBreakUntriggeredAction;
import com.team1816.season.subsystems.Ramp;

public class OuttakeL1Mode extends AutoMode {

    public OuttakeL1Mode() {}

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
                new SeriesAction(
                        new RampQuickAction(Ramp.RAMP_STATE.SCORE),
                        new WaitAction(3),
                        new SetFeederStatesAction(false)
                )
        );
    }
}
