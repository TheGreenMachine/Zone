package com.team1816.lib.auto.modes;

import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * An abstract class that is the basis of a robot's autonomous routines.
 * Actions can be implemented in the routine and can be performed  (which are routines that do actions).
 */
public abstract class AutoMode implements Runnable {
    public AutoMode(String name) {
        this.name = name;
    }

    public static RobotState robotState;

    private static final long looperDtInMS = (long) (Constants.kLooperDt * 1000);
    
    private final String name;

    /**
     * State: if mode needs to be stopped
     */
    private boolean needsStop;
    /**
     * State: Initial Pose that robot starts at
     */
    protected Pose2d initialPose;

    /**
     * Runs the autoMode routine actions
     *
     * @see #routine()
     */
    public void run() {
        start();

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE STOPPED EARLY ! ! !", false);
        }

        done();
    }

    /**
     * Starts the AutoMode and relevant actions
     */
    private void start() {
        GreenLogger.log("Starting " + this.getClass().getName());
        needsStop = false;
    }

    /**
     * Routine register of actions that will be run in the mode
     *
     * @throws AutoModeEndedException
     */
    protected abstract void routine() throws AutoModeEndedException;

    /**
     * Standard cleanup end-procedure
     */
    protected void done() {
        GreenLogger.log(this.getClass().getName() + " Done");
    }

    /**
     * Stops the auto mode
     */
    public void stop() {
        needsStop = true;
    }

    /**
     * Runs a given action, typically placed in routine()
     *
     * @param action
     * @throws AutoModeEndedException
     * @see AutoAction
     */
    protected void runAction(AutoAction action) throws AutoModeEndedException {
        action.start();

        // Run action, stop action on interrupt or done
        while (!action.isFinished()) {
            if (needsStop) {
                throw new AutoModeEndedException();
            }

            action.update();

            try {
                Thread.sleep(looperDtInMS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

    /**
     * Returns the initial pose of the robot
     *
     * @return initialPose
     */
    public Pose2d getInitialPose() {
        if (initialPose == null) {
            return Constants.kDefaultZeroingPose;
        }
        return initialPose;
    }

    public Pose2d getInitialPose(Color allianceColor) {
        return getInitialPose();
    }
    
    public String getName() {
        return name;
    }
    
    @Override
    public String toString() {
        return "AutoMode{" +
                "name='" + name + '\'' +
                ", initialPose=" + initialPose +
                '}';
    }
}
