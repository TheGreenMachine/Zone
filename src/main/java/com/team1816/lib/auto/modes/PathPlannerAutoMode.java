package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.PathPlannerAction;
import edu.wpi.first.math.geometry.Pose2d;

public class PathPlannerAutoMode extends AutoMode {
    public PathPlannerAutoMode(String autoName) {
        super(autoName);
        action = new PathPlannerAction(autoName, PathPlannerAction.ActionType.AUTO);
    }
    
    public PathPlannerAutoMode(String autoName, boolean mirror) {
        super(autoName);
        action = new PathPlannerAction(autoName, PathPlannerAction.ActionType.AUTO, mirror);
    }
    
    private final PathPlannerAction action;
    
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(action);
    }
    
    @Override
    public Pose2d getInitialPose() {
        return action.getPathInitialPose();
    }
}
