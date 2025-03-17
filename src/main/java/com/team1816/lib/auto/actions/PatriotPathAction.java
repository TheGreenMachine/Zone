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
 * follow a PathPlanner auto, or dynamically generate a patriot path
 */
public class PatriotPathAction implements AutoAction {
    private final Pose2d initialPose;
    private final Command pathCommand;
    private final Drive drive;
    private boolean valid;
    
    /**
     * Creates a {@link PathPlannerAction} by dynamically creating one.
     * <p>
     * Visit <a href="https://pathplanner.dev/">pathplanner.dev</a> for more information.
     */
    public PatriotPathAction(Pose2d initialPose, Pose2d targetPose, double endVelocity) {
        this.initialPose = initialPose;
        this.drive = Injector.get(Drive.Factory.class).getInstance();
        this.valid = true;
        
        if (drive instanceof TankDrive) {
            GreenLogger.log("Tank Drive is not supported by our PathPlanner implementation.");
            this.pathCommand = null;
            this.valid = false;
        } else if (drive instanceof EnhancedSwerveDrive) {
            PathConstraints constraints = new PathConstraints( // TODO: change this to actual values
                    3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
            this.pathCommand = AutoBuilder.pathfindToPose(
                    targetPose, constraints, endVelocity
            );
        } else {
            GreenLogger.log(
                    " oh man oh god I'm neither swerve nor tank! " + drive.toString()
            );
            this.pathCommand = null;
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
}
