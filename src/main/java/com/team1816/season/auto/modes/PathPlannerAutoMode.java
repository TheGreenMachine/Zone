package com.team1816.season.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.PathPlannerAction;
import com.team1816.lib.auto.modes.AutoMode;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public class PathPlannerAutoMode extends AutoMode {
    private final PathPlannerAction action;

    public PathPlannerAutoMode(String actionName) {
        this(actionName, false);
    }

    public PathPlannerAutoMode(String name, boolean mirror) {
        super();

        action = new PathPlannerAction(name, PathPlannerAction.ActionType.AUTO, mirror);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(action);
    }

    @Override
    public List<Pose2d> getPoses() {
        if (AutoBuilder.shouldFlip()) {
            return action.getBluePoses().stream()
                    .map(FlippingUtil::flipFieldPose)
                    .toList();
        } else {
            return action.getBluePoses();
        }
    }

    @Override
    public Pose2d getInitialPose() {
        return action.getPathInitialPose();
    }
}
