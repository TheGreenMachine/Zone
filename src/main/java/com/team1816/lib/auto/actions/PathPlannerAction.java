package com.team1816.lib.auto.actions;

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

/**
 * This class represents an action that will move the robot along a PathPlanner trajectory.
 */
public class PathPlannerAction implements AutoAction {
    private final Pose2d initialPose;
    private final Command pathCommand;
    private final Drive drive;

    // list of all poses of action from blue perspective for logging purposes
    private final List<Pose2d> bluePoses;

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
     * @throws RuntimeException when the path failed to load
     */
    public PathPlannerAction(String actionName, ActionType actionType, boolean mirror) {
        this.drive = Injector.get(Drive.Factory.class).getInstance();

        if (drive instanceof TankDrive) {
            GreenLogger.log("Tank Drive is not supported by our PathPlanner implementation.");
            this.pathCommand = null;
            this.initialPose = null;
            bluePoses = List.of();
        } else if (drive instanceof EnhancedSwerveDrive) {
            switch (actionType) {
                case PATH -> {
                    PathPlannerPath path;
                    try {
                        path = PathPlannerPath.fromPathFile(actionName);
                    } catch (Exception e) {
                        throw new RuntimeException(e);
                    }

                    if (mirror) path = path.flipPath();

                    this.pathCommand = AutoBuilder.followPath(path);
                    this.initialPose = path.getPathPoses().get(0);
                    this.bluePoses = path.getPathPoses();
                }

                case AUTO -> {
                    this.pathCommand = new PathPlannerAuto(actionName, mirror);
                    this.initialPose = ((PathPlannerAuto) pathCommand).getStartingPose();

                    List<Pose2d> poses;

                    try {
                        poses = PathPlannerAuto.getPathGroupFromAutoFile(actionName).stream()
                                .map(PathPlannerPath::getPathPoses)
                                .flatMap(Collection::stream)
                                .collect(Collectors.toList());

                        if (mirror) {
                            poses.replaceAll(PathPlannerAction::mirrorPose);
                        }

                        System.out.println("Constructor poses: " + poses);
                    } catch (Exception ignored) {
                        GreenLogger.log("Could not get auto poses for " + actionName + "!");
                        poses = List.of();
                    }

                    this.bluePoses = poses;
                }

                default -> throw new IllegalStateException("Unknown action type: " + actionType);
            }
        } else {
            GreenLogger.log(
                    " oh man oh god I'm neither swerve nor tank! " + drive.toString()
            );
            initialPose = null;
            pathCommand = null;
            bluePoses = List.of();
        }
    }

    /**
     * Returns the initial pose of the action.
     */
    public Pose2d getPathInitialPose() {
        return AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(initialPose) : initialPose;
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

    /**
     * @return a list of non-flipped poses that the action will follow for logging purposes
     */
    public List<Pose2d> getBluePoses() {
        return bluePoses;
    }

    public enum ActionType {
        PATH, AUTO
    }

    private static Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(pose.getX(), FlippingUtil.fieldSizeY -  pose.getY(), pose.getRotation());
    }
}
