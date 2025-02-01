package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.season.subsystems.CoralArm;

public class PlaceCoralSeriesAction extends SeriesAction{
    public PlaceCoralSeriesAction() {
        super(
                new SeriesAction(
                        new CoralArmIntakeAction(CoralArm.INTAKE_STATE.OUTTAKE),
                        new WaitAction(1)
                )
        );
    }
}
