package com.team1816.lib.autopath;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.PathPlannerAction;
import com.team1816.lib.auto.actions.PatriotPathAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import org.json.simple.parser.ParseException;

import java.io.IOException;

@Singleton
public class PatriotPath {
    public RobotState robotState;
    
    private PatriotPathAction pathPlannerAction;

    /**
     * State: if path needs to be stopped
     */
    private boolean needsStop;
    
    @Inject
    public PatriotPath() {
        robotState = Injector.get(RobotState.class);
    }

    public void start(Pose2d targetPosition) {
        if (robotState.autopathing && (double) System.nanoTime() / 1000000 - robotState.autopathBeforeTime < robotState.autopathPathCancelBufferMilli)
            return;
        robotState.autopathing = true;

        GreenLogger.log("Starting Patriot Path");
        needsStop = false;

        robotState.autopathBeforeTime = (double) System.nanoTime() / 1000000;

        pathPlannerAction = new PatriotPathAction(robotState.fieldToVehicle, targetPosition);

        pathPlannerAction.start();
    }

    public void start(String resultPathName) {
        if (robotState.autopathing && (double) System.nanoTime() / 1000000 - robotState.autopathBeforeTime < robotState.autopathPathCancelBufferMilli)
            return;
        robotState.autopathing = true;

        GreenLogger.log("Starting Patriot Path");
        needsStop = false;

        try {
            pathPlannerAction = new PatriotPathAction(PathPlannerPath.fromPathFile(resultPathName));
        } catch (IOException | ParseException e) {
            GreenLogger.log("Failed to load PathPlanner path! Defaulting to no-op path.");
            GreenLogger.log(e);
            pathPlannerAction = new PatriotPathAction(robotState.fieldToVehicle);
        }

        pathPlannerAction.start();
    }

    /**
     * Called every loop
     */
    public void routine() throws AutoModeEndedException {
        // Run actions here:
        // e.g. runAction(new SeriesAction(new WaitAction(0.5), ...))

        // Run action, stop action on interrupt or done
        if (!pathPlannerAction.isFinished()) {
            if (needsStop) {
                throw new AutoModeEndedException();
            }

            pathPlannerAction.update();
        } else {
            pathPlannerAction.done();
            done();
        }
    }

    /**
     * Standard cleanup end-procedure
     */
    protected void done() {
        robotState.autopathing = false;
    }

    /**
     * Stops the auto path
     */
    public void stop() {
        pathPlannerAction.done();
        robotState.autopathing = false;
    }
}
