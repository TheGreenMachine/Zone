package com.team1816.lib.auto.modes;

import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.PathPlannerAction;
import edu.wpi.first.math.geometry.Pose2d;

public class PathPlannerAutoMode extends AutoMode {
    public PathPlannerAutoMode(String autoName) {
        super(autoName);
        action = new PathPlannerAction(autoName, PathPlannerAction.ActionType.AUTO);
        super.initialPose = action.getPathInitialPose();
    }
    
    public PathPlannerAutoMode(String autoName, boolean mirror) {
        super(autoName);
        action = new PathPlannerAction(autoName, PathPlannerAction.ActionType.AUTO, mirror);
        super.initialPose = action.getPathInitialPose();
    }
    
    private final PathPlannerAction action;
    
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(action);
    }

    @Override
    public Pose2d getInitialPose(Color allianceColor) {
        return (allianceColor == Color.BLUE) ? getInitialPose() : FlippingUtil.flipFieldPose(getInitialPose());
    }
}
