package com.team1816.lib.autopath;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.PathPlannerAction;
import com.team1816.lib.auto.actions.PatriotPathAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;

@Singleton
public class PatriotPath {
    public RobotState robotState;
    
    private PatriotPathAction pathPlannerAction;

    /**
     * State: if path needs to be stopped
     */
    private boolean needsStop;

    /**
     * Initializes Autopath
     */
    @Inject
    public PatriotPath() {
        robotState = Injector.get(RobotState.class);
    }

    /**
     * Starts the Autopath and relevant actions
     */
    public void start(Pose2d autopathTargetPosition) {
        if (robotState.autopathing && (double) System.nanoTime() / 1000000 - robotState.autopathBeforeTime < robotState.autopathPathCancelBufferMilli)
            return;
        robotState.autopathing = true;

        GreenLogger.log("Starting Autopath");
        needsStop = false;

        robotState.autopathBeforeTime = (double) System.nanoTime() / 1000000;

        pathPlannerAction = new PatriotPathAction(robotState.fieldToVehicle, autopathTargetPosition, 0);

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
