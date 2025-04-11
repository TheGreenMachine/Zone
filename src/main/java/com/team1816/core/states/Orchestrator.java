package com.team1816.core.states;

import com.ctre.phoenix6.StatusCode;
import com.team1816.lib.Singleton;
import com.team1816.lib.Inject;
import com.team1816.core.configuration.Constants;
import com.team1816.core.configuration.FieldConfig;
import com.team1816.lib.Injector;
import com.team1816.lib.PlaylistManager;
import com.team1816.lib.input_handler.InputHandler;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.vision.Camera;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import com.team1816.season.subsystems.Ramp;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import static com.team1816.lib.subsystems.Subsystem.robotState;

/**
 * Main superstructure-style class and logical operator for handling and delegating subsystem tasks. Consists of an integrated
 * drivetrain with other subsystems and utilizes closed loop state dependent control via {@link RobotState}.
 *
 * @see RobotState
 */
@Singleton
public class Orchestrator {

    /**
     * Subsystems
     */
    private Drive drive;
    private Camera camera;
    private LedManager ledManager;
    //TODO add new subsystems here
    private InputHandler inputHandler;
    private CoralArm coralArm;
    private Elevator elevator;
    private Ramp ramp;

    /**
     * Properties
     */
    private static boolean rumbleStopped;

    // Place threads here.
    // e.g. private Thread [ThreadName]Thread;

    public static boolean runningAutoTarget = false;

    // Place appropriate running booleans here.
    // e.g. public static boolean running[ThreadName] = false;

    /**
     * Instantiates an Orchestrator with all its subsystems
     *
     * @param df  Drive.Factory (derives drivetrain)
     * @param led LedManager
     */
    @Inject
    public Orchestrator(Drive.Factory df, Camera cam, LedManager led) {
        /**
         * Insert any other parameters into the constructor if you need to
         * manage them.
         *
         * e.g. a Subsystem of some kind like the LEDManager.
         */

        drive = df.getInstance();
        camera = cam;
        ledManager = led;
        //TODO init new subsystems here
        inputHandler = Injector.get(InputHandler.class);
        coralArm = Injector.get(CoralArm.class);
        elevator = Injector.get(Elevator.class);
        ramp = Injector.get(Ramp.class);
    }

    /**
     * Actions
     */

    // Place any actions here.

    /**
     * Music Control
     */

    //TODO Move orchestra object out of drive and add wrapper - LOW LOW PRIORITY

    /**
     * Plays/Pauses the selected song for the Orchestra
     * @see com.ctre.phoenix6.Orchestra
     * @param playing If the song should play or pause
     */
    public void playSong(boolean playing) {
        if (playing) {
            if (!drive.orchestra.isPlaying()) {
                drive.orchestra.play();
            }
        } else {
            drive.orchestra.pause();
        }
    }

    /**
     * Stops the Orchestra from playing a song
     * @see com.ctre.phoenix6.Orchestra
     */
    public void stopSong() {
        drive.orchestra.stop();
    }

    /**
     * Loads the entered song's filepath into the Orchestra
     *
     * @param song The selected song
     * @see com.ctre.phoenix6.Orchestra
     */
    public StatusCode loadSong(PlaylistManager.Playlist song) {
        return loadSong(song.getFilePath());
    }

    /**
     * Loads the entered filepath into the Orchestra
     *
     * @param filepath
     * @see com.ctre.phoenix6.Orchestra
     */
    private StatusCode loadSong(String filepath) {
        return drive.orchestra.loadMusic(filepath);
    }

    /**
     * Clears executable threads
     */
    public void clearThreads() {
        /**
            For clearing a thread, here is the general pattern we follow:

            if (thread != null && thread.isAlive()) {
                thread.stop();
            }

            Make sure to use the pattern above to avoid causing exceptions
            and any errors, when stopping the work on a thread.
         */
    }

    /**
     *Updates the robot pose with vision data
     */
    public void updatePoseWithVisionData() {
        camera.updateVisionEstimatedPoses();
        for (int i = 0; i < robotState.visionEstimatedPoses.size(); i++) {
            drive.updateOdometryWithVision(
                    robotState.visionEstimatedPoses.get(i).estimatedPose.toPose2d(),
                    robotState.visionEstimatedPoses.get(i).timestampSeconds,
                    robotState.visionStdDevs.get(i)
            );
        }
    }

    public void setFeederStates(boolean setToL1Feeder) {
        coralArm.setDesiredState(CoralArm.PIVOT_STATE.FEEDER, setToL1Feeder ? CoralArm.INTAKE_STATE.HOLD : CoralArm.INTAKE_STATE.INTAKE);
        elevator.setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
        ramp.setDesiredState(setToL1Feeder ? Ramp.RAMP_STATE.L1_FEEDER : Ramp.RAMP_STATE.L234_FEEDER);
    }

    //Just a wrapper to keep paradigm
    public void setControllerRumble(InputHandler.ControllerRole controller, InputHandler.RumbleDirection rumbleDirection, double rumbleLevel) {
        rumbleStopped = rumbleLevel == 0;
        inputHandler.setRumble(controller, rumbleDirection, rumbleLevel);
    }

    public void stopRumble(InputHandler.ControllerRole controller) {
        if (!rumbleStopped) {
            setControllerRumble(controller, InputHandler.RumbleDirection.UNIFORM, 0);
        }
    }
}
