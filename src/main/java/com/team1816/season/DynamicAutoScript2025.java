package com.team1816.season;

import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.Consumer;

public class DynamicAutoScript2025 {
    private ArrayList<SendableChooser<TrajectoryAction>> trajectoryActions = new ArrayList<>();
    private ArrayList<TrajectoryAction> autoTrajectoryActions = new ArrayList<>();

    public DynamicAutoScript2025(int numOfTrajectoryActions){
        for(int i = 0; i < numOfTrajectoryActions; i++)
            trajectoryActions.add(new SendableChooser<>());

        Consumer<ArrayList<SendableChooser<TrajectoryAction>>> updateTrajectoryActions = actions ->
        {
            for (int i = 0; i < actions.size(); i++) {
                autoTrajectoryActions.set(i, actions.get(i).getSelected());
            }
        };
        for(SendableChooser action : trajectoryActions){
            action.onChange(updateTrajectoryActions);
        }
    }

    public ArrayList<TrajectoryAction> getAutoTrajectoryActions(){
        return autoTrajectoryActions;
    }

    private void addSendableChooserOption(String name, AutoMode mode){
        for(SendableChooser action : trajectoryActions)
            action.addOption(name, mode);
    }

    private void addToSmartDashboard(){
        for(int i = 0; i < trajectoryActions.size(); i++)
            SmartDashboard.putData("TrajectoryAction "+i, trajectoryActions.get(i));
    }
}
