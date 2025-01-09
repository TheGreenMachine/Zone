package com.team1816.season.auto;

import com.team1816.lib.auto.modes.AutoMode;

import java.util.ArrayList;

public class DynamicAutoModeManager implements Runnable{
    private ArrayList<AutoMode> autoModes;

    public DynamicAutoModeManager(ArrayList<AutoMode> autoModes){
        this.autoModes = autoModes;
    }

    @Override
    public void run() {
        for(AutoMode autoMode : autoModes){
            autoMode.run();
        }
    }
}
