package com.team1816.lib.auto.actions;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.EnhancedSwerveDrive;
import com.team1816.lib.subsystems.drive.TankDrive;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;

/**
 * This class represents an action that will move the robot along a PathPlanner trajectory,
 * follow a PathPlanner auto, or dynamically generate a patriot path
 */
public class PatriotPathAction implements AutoAction {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(.5, 1, Units.degreesToRadians(720), Units.degreesToRadians(540));

    private static final double ADSTAR_MIN_THRESHOLD = 2; // in metres; an arbitrary value we use to determine when PathPlanner's AD* path will work

    private final Pose2d initialPose;
    private final Command pathCommand;
    private final Drive drive;
    private boolean valid;

    /**
     * Creates a no-op Patriot Path.
     */
    public PatriotPathAction(Pose2d initialPose) {
        this.initialPose = initialPose;
        this.drive = Injector.get(Drive.Factory.class).getInstance();
        this.valid = false;
        this.pathCommand = Commands.none();
    }

    /**
     * Creates a {@link PathPlannerAction} that paths from initialPose to targetPose
     * <p>
     * Visit <a href="https://pathplanner.dev/">pathplanner.dev</a> for more information.
     */
    public PatriotPathAction(Pose2d initialPose, Pose2d targetPose) {
        this.initialPose = initialPose;
        this.drive = Injector.get(Drive.Factory.class).getInstance();
        this.valid = true;

        if (initialPose.equals(targetPose)) {
            this.pathCommand = Commands.none();
        } else if (drive instanceof TankDrive) {
            GreenLogger.log("Tank Drive is not supported by our PathPlanner implementation.");
            this.pathCommand = Commands.none();
            this.valid = false;
        } else if (drive instanceof EnhancedSwerveDrive) {
            double dist = initialPose.getTranslation().getDistance(targetPose.getTranslation());
            if (dist < ADSTAR_MIN_THRESHOLD) {
                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(initialPose, targetPose);
                this.pathCommand = AutoBuilder.followPath(
                        new PathPlannerPath(
                                waypoints,
                                CONSTRAINTS,
                                new IdealStartingState(Injector.get(RobotState.class).robotVelocity, initialPose.getRotation()),
                                new GoalEndState(0, targetPose.getRotation())
                        ));
            } else {
                this.pathCommand = AutoBuilder.pathfindToPose(targetPose, CONSTRAINTS);
            }
        } else {
            GreenLogger.log(
                    " oh man oh god I'm neither swerve nor tank! " + drive.toString()
            );
            this.pathCommand = Commands.none();
            this.valid = false;
        }
    }

    /**
     * Creates a {@link PathPlannerAction} that pathfinds to the origin of a path and follows that
     * path. Useful for precise aiming and lining up.
     * <p>
     * Visit <a href="https://pathplanner.dev/">pathplanner.dev</a> for more information.
     */
    public PatriotPathAction(PathPlannerPath path) {
        this.initialPose = Injector.get(RobotState.class).fieldToVehicle;
        this.drive = Injector.get(Drive.Factory.class).getInstance();
        this.valid = true;

        if (drive instanceof TankDrive) {
            GreenLogger.log("Tank Drive is not supported by our PathPlanner implementation.");
            this.pathCommand = null;
            this.valid = false;
        } else if (drive instanceof EnhancedSwerveDrive) {
            this.pathCommand = AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);
        } else {
            GreenLogger.log(
                    " oh man oh god I'm neither swerve nor tank! " + drive.toString()
            );
            this.pathCommand = Commands.none();
            this.valid = false;
        }
    }


    /**
     * Returns the initial pose of the action. Note that this does not change based on the color
     * of the alliance.
     */
    public Pose2d getPathInitialPose() {
        return initialPose;
    }

    /**
     * Starts the command, executes trajectory on drivetrain
     *
     * @see Drive#startTrajectory(Trajectory, List)
     * @see Command
     * @see AutoAction#start()
     */
    @Override
    public void start() {
        if (!valid) {
            GreenLogger.log("Attempted to run an invalid PathPlanner action!");
            return;
        }

        drive.setControlState(Drive.ControlState.TRAJECTORY_FOLLOWING);
        pathCommand.initialize();
    }

    /**
     * Executes the command
     *
     * @see Command
     * @see Command#execute()
     * @see AutoAction#update()
     */
    @Override
    public void update() {
        pathCommand.execute();
    }

    /**
     * Returns whether or not the command has been executed
     *
     * @return boolean isFinished
     * @see Command
     * @see AutoAction#isFinished()
     */
    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    /**
     * Ends the command, stops drivetrain
     *
     * @see Drive
     * @see Command
     * @see AutoAction#done()
     */
    @Override
    public void done() {
        pathCommand.end(false);
        drive.stop();
    }
}
