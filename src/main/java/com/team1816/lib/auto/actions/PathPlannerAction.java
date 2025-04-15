package com.team1816.lib.auto.actions;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.lib.Injector;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.EnhancedSwerveDrive;
import com.team1816.lib.subsystems.drive.TankDrive;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;

/**
 * This class represents an action that will move the robot along a PathPlanner trajectory,
 * follow a PathPlanner auto
 */
public class PathPlannerAction implements AutoAction {
    private Pose2d initialPose;
    private final Command pathCommand;
    private final Drive drive;
    private boolean valid;
    
    /**
     * Creates a {@link PathPlannerAction} by loading a {@link PathPlannerPath} from the deploy
     * folder.
     * <p>
     * Visit <a href="https://pathplanner.dev/">pathplanner.dev</a> for more information.
     *
     * @param actionType Whether to load an auto or a path
     * @throws RuntimeException when the path failed to load
     */
    public PathPlannerAction(String actionName, ActionType actionType) {
        this(actionName, actionType, false);
    }
    
    /**
     * Creates a {@link PathPlannerAction} by loading a {@link PathPlannerPath} from the deploy
     * folder.
     * <p>
     * Visit <a href="https://pathplanner.dev/">pathplanner.dev</a> for more information.
     *
     * @param actionType Whether to load an auto or a path
     * @param mirror     Whether to mirror the path
     * @throws RuntimeException when the path failed to load
     */
    public PathPlannerAction(String actionName, ActionType actionType, boolean mirror) {
        this.drive = Injector.get(Drive.Factory.class).getInstance();
        this.valid = true;
        
        if (drive instanceof TankDrive) {
            GreenLogger.log("Tank Drive is not supported by our PathPlanner implementation.");
            this.pathCommand = Commands.none();
            this.initialPose = Pose2d.kZero;
            this.valid = false;
        } else if (drive instanceof EnhancedSwerveDrive) {
            switch (actionType) {
                case PATH -> {
                    PathPlannerPath path;
                    try {
                        path = PathPlannerPath.fromPathFile(actionName);
                    } catch (Exception e) {
                        GreenLogger.log("Could not load path " + actionName + "!");
                        this.pathCommand = Commands.none();
                        this.initialPose = Pose2d.kZero;
                        this.valid = false;
                        return;
                    }
                    
                    if (mirror) path = path.mirrorPath();
                    
                    this.pathCommand = AutoBuilder.followPath(path);
                    this.initialPose = path.getPathPoses().get(0);
                }
                
                case AUTO -> {
                    this.pathCommand = new PathPlannerAuto(actionName, mirror);
                    this.initialPose = ((PathPlannerAuto) pathCommand).getStartingPose();
                    
                    // if the auto failed to load, its initial rotation will be null, making it an invalid pose.
                    // this is a failsafe preventing a complete crash of the robot.
                    try {
                        this.initialPose.getRotation();
                    } catch (NullPointerException ignored) {
                        this.initialPose = Pose2d.kZero;
                        this.valid = false;
                    }
                }
                
                default -> throw new IllegalStateException("Unknown action type: " + actionType);
            }
        } else {
            GreenLogger.log(
                    " oh man oh god I'm neither swerve nor tank! " + drive.toString()
            );
            this.initialPose = Pose2d.kZero;
            this.pathCommand = Commands.none();
            this.valid = false;
        }
    }

    
    /**
     * Returns the initial pose of the action. Note that this does not change based on the colour
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
    
    public enum ActionType {
        PATH, AUTO
    }
}
