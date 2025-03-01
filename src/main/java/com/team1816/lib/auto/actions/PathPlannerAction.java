package com.team1816.lib.auto.actions;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.EnhancedSwerveDrive;
import com.team1816.lib.subsystems.drive.TankDrive;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

/**
 * This class represents an action that will move the robot along a PathPlanner trajectory.
 */
public class PathPlannerAction extends TrajectoryAction {
    /**
     * Creates a {@link PathPlannerAction} by loading a {@link PathPlannerPath} from the deploy
     * folder.
     * <p>
     * Visit <a href="https://pathplanner.dev/">pathplanner.dev</a> for more information.
     *
     * @throws RuntimeException when the path failed to load
     */
    public PathPlannerAction(String actionName) {
        super(new Trajectory(), List.of()); // TODO: uncouple AutoModes from TrajectoryAction ASAP
        try {
            this.path = PathPlannerPath.fromPathFile(actionName);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        this.drive = Injector.get(Drive.Factory.class).getInstance();

        if (drive instanceof TankDrive) {
            GreenLogger.log("Tank Drive is no longer supported.");
            pathCommand = null;
        } else if (drive instanceof EnhancedSwerveDrive) {
            this.pathCommand = AutoBuilder.followPath(path);
        } else {
            GreenLogger.log(
                    " oh man oh god I'm neither swerve nor tank! " + drive.toString()
            );
            pathCommand = null;
        }

        robotState = Injector.get(RobotState.class);
    }

    private final PathPlannerPath path;
    private final Command pathCommand;
    private final Drive drive;

    private final RobotState robotState;

    /**
     * Returns the path that is associated with the action
     *
     * @return trajectory
     * @see Trajectory
     */
    public PathPlannerPath getPath() {
        return path;
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
        drive.setControlState(Drive.ControlState.TRAJECTORY_FOLLOWING);
        pathCommand.initialize();

//        RobotConfig config;
//        try {
//            config = RobotConfig.fromGUISettings();
//        } catch (IOException | ParseException e) {
//            throw new RuntimeException(e);
//        }
//        var sm = path.generateTrajectory(robotState.robotChassis, robotState.fieldToVehicle.getRotation(), config).getStates();
//
//        System.out.println(sm);
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
