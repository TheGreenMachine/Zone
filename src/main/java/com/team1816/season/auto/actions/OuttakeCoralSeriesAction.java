package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.season.subsystems.CoralArm;

public class OuttakeCoralSeriesAction extends SeriesAction{
    public OuttakeCoralSeriesAction() {
        super(
                new SeriesAction(
                        new CoralArmQuickIntakeAction(CoralArm.INTAKE_STATE.OUTTAKE),
                        new WaitAction(1)
                )
        );
    }
}
