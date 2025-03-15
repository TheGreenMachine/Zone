package com.team1816.season.auto.actions;

import com.team1816.core.states.Orchestrator;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;

public class SetFeederStatesAction implements AutoAction {
    private Orchestrator orchestrator;
    private boolean setToL1Feeder;

    public SetFeederStatesAction(boolean setToL1Feeder){
        orchestrator = Injector.get(Orchestrator.class);
        this.setToL1Feeder = setToL1Feeder;
    }

    @Override
    public void start() {
        orchestrator.setFeederStates(setToL1Feeder);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
