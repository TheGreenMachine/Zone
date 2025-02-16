package com.team1816.core;

import com.ctre.phoenix6.CANBus;
import com.team1816.core.auto.AutoModeManager;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.Orchestrator;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.PlaylistManager;
import com.team1816.lib.auto.Color;
//import com.team1816.lib.autopath.Autopath;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.input_handler.InputHandler;
import com.team1816.lib.input_handler.controlOptions.ActionState;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.SubsystemLooper;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.vision.Camera;
import com.team1816.lib.util.Util;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.DynamicAutoScript2025;
import com.team1816.season.subsystems.AlgaeCatcher;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import com.team1816.season.subsystems.Pneumatic;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

public class Robot extends TimedRobot {
    //TODO remove this variable
    private boolean isLooping = false;

    /**
     * Looper
     */
    private final Looper enabledLoop;
    private final Looper disabledLoop;

    /**
     * Controls
     */
    private InputHandler inputHandler;

    private Infrastructure infrastructure;
    private SubsystemLooper subsystemManager;

    /**
     * State Managers
     */
    private Orchestrator orchestrator;
    private RobotState robotState;

    private PlaylistManager playlistManager;
    private boolean desireToPlaySong;

    /**
     * Subsystems
     */
    private Drive drive;

    //TODO add new subsystems here
//    private Autopath autopather;
    private Elevator elevator;
    private CoralArm coralArm;
    private AlgaeCatcher algaeCatcher;
    private Pneumatic pneumatic;

    private LedManager ledManager;
    private Camera camera;

    private DynamicAutoScript2025 dynamicAutoScript;

    /**
     * Factory
     */
    private static RobotFactory factory;

    /**
     * Autonomous
     */
    private AutoModeManager autoModeManager;

    private Thread autoTargetAlignThread;

    /**
     * Timing
     */
    private double loopStart;
    public static double looperDt; //looptime delta
    public static double robotDt;
    public static double autoStart;
    public static double teleopStart;

    private DoubleLogEntry robotLoopLogger;
    private DoubleLogEntry looperLogger;
    private DoubleLogEntry canivoreTrafficLogger;
    private DoubleLogEntry lowSpeedTrafficLogger;


    /**
     * Properties
     */
    private boolean faulted;


    /**
     * Instantiates the Robot by injecting all systems and creating the enabled and disabled loopers
     */
    Robot() {
        super();
        // initialize injector
        enabledLoop = new Looper(this);
        disabledLoop = new Looper(this);

        desireToPlaySong = false;

        if (Constants.kLoggingRobot) {
            robotLoopLogger = new DoubleLogEntry(DataLogManager.getLog(), "Timings/Robot");
            looperLogger = new DoubleLogEntry(DataLogManager.getLog(), "Timings/RobotState");

            if (Constants.kHasCANivore) {
                canivoreTrafficLogger = new DoubleLogEntry(DataLogManager.getLog(), "CAN/highSpeedUtilization");
            }
            lowSpeedTrafficLogger = new DoubleLogEntry(DataLogManager.getLog(), "CAN/lowSpeedUtilization");
        }
    }

    /**
     * Returns the length of the last loop that the Robot was on
     *
     * @return duration (ms)
     */
    public Double getLastRobotLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    /**
     * Returns the duration of the last enabled loop
     *
     * @return duration (ms)
     * @see Looper#getLastLoop()
     */
    public Double getLastSubsystemLoop() {
        return enabledLoop.isRunning() ? enabledLoop.getLastLoop() : disabledLoop.getLastLoop();
    }

    /**
     * Actions to perform when the robot has just begun being powered and is done booting up.
     * Initializes the robot by injecting the controlboard, and registering all subsystems.
     */
    @Override
    public void robotInit() {
        try {
            /** Register All Subsystems */
            DriverStation.silenceJoystickConnectionWarning(true);
            // Remember to register our subsystems below! The subsystem manager deals with calling
            // readFromHardware and writeToHardware on a loop, but it can only call read/write it if it
            // can recognize the subsystem. To recognize your subsystem, just add it alongside the
            // drive, ledManager, and camera parameters.


            // TODO: Set up any other subsystems here.

            factory = Injector.get(RobotFactory.class);
            ledManager = Injector.get(LedManager.class);
            camera = Injector.get(Camera.class);
            camera.setDriverMode(true);
            robotState = Injector.get(RobotState.class);
            orchestrator = Injector.get(Orchestrator.class);
            infrastructure = Injector.get(Infrastructure.class);
            subsystemManager = Injector.get(SubsystemLooper.class);
            autoModeManager = Injector.get(AutoModeManager.class);
            playlistManager = Injector.get(PlaylistManager.class);
//            autopather = Injector.get(Autopath.class);
            coralArm = Injector.get(CoralArm.class);
            elevator = Injector.get(Elevator.class);
            algaeCatcher = Injector.get(AlgaeCatcher.class);
            pneumatic = Injector.get(Pneumatic.class);

            dynamicAutoScript = new DynamicAutoScript2025(5, 3);

            /** Logging */
            if (Constants.kLoggingRobot) {
                var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
                var robotName = System.getenv("ROBOT_NAME");
                if (robotName == null) robotName = "default";
                var logFileDir = "/home/lvuser/";
                String OS_NAME = System.getProperty("os.name").toLowerCase();
                // if there is a USB drive use it
                if (Files.exists(Path.of("/media/sda1"))) {
                    logFileDir = "/media/sda1/";
                }

//                // Characterize SignalLogger
//                SignalLogger.setPath(logFileDir);
//                SignalLogger.enableAutoLogging(DriverStation.isFMSAttached());

                if (RobotBase.isSimulation()) {
                    if (OS_NAME.contains("win")) {
                        logFileDir = System.getenv("temp") + "\\";
                    } else {
                        logFileDir = System.getProperty("user.dir") + "/";
                    }
                    if (!OS_NAME.contains("mac")) { //Can't open .hoot on mac so won't clog logs up in sim
//                        SignalLogger.start();
                    }
                }

                // start logging
                DataLogManager.start(logFileDir, "", Constants.kLooperDt);
                if (RobotBase.isReal()) {
                    if (!DriverStation.isFMSAttached()) {
                        Util.cleanLogFiles();
                    }
//                    SignalLogger.start();
                }
                DriverStation.startDataLog(DataLogManager.getLog(), false);
            }

            drive = (Injector.get(Drive.Factory.class)).getInstance();

            subsystemManager.setSubsystems(drive, ledManager, camera, coralArm, elevator, algaeCatcher, pneumatic);

            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);

            subsystemManager.zeroSensors();
            // zeroing ypr - (-90) pigeon is mounted with the "y" axis facing forward
            drive.resetPigeon(Rotation2d.fromDegrees(-90));


            /** [Specific subsystem] not zeroed on boot up - letting ppl know */
            faulted = true;

            SmartDashboard.putBoolean("PlaySong", false);

            RobotController.setBrownoutVoltage(5.5);

            /** Register inputHandler */
            inputHandler = Injector.get(InputHandler.class);


            /** Driver Commands */
            inputHandler.listenAction(
                    "zeroPose",
                    ActionState.PRESSED,
                    () ->
                            drive.resetHeading(
                                    robotState.allianceColor == Color.BLUE ?
                                            Rotation2d.fromDegrees(0) :
                                            Rotation2d.fromDegrees(180)
                            )
            );

            inputHandler.listenAction(
                    "hardZeroPose", //FIXME this sometimes does weird rotation in sim, idk about in real life
                    ActionState.PRESSED,
                    () ->
                            drive.zeroSensors(
                                    robotState.allianceColor == Color.BLUE ?
                                            Constants.kDefaultZeroingPose :
                                            Constants.kFlippedZeroingPose
                            )
            );


            inputHandler.listenActionPressAndRelease(
                    "brakeMode",
                    drive::setBraking
            );

            inputHandler.listenActionPressAndRelease(
                    "turboMode",
                    drive::setTurboMode
            );

            inputHandler.listenActionPressAndRelease(
                    "slowMode",
                    drive::setSlowMode
            );

            /*inputHandler.listenAction(
                    "autopathingSpeaker",
                    ActionState.PRESSED,
                    () ->
                        autopather.start(new Pose2d(new Translation2d(1.6, 5.5), Rotation2d.fromDegrees(0)))
            );

            inputHandler.listenAction(
                    "autopathingAmp",
                    ActionState.PRESSED,
                    () ->
                        autopather.start(new Pose2d(new Translation2d(15.2, 1.1), Rotation2d.fromDegrees(135)))
            );*/
            /*NEW SUBSYSTEM ACTIONS*/
            /*inputHandler.listenActionPressAndRelease(
                    "intakeCoral",
                    (pressed) -> {
                        coralArm.setDesiredIntakeState((pressed && !CoralArm.robotState.isCoralBeamBreakTriggered) ? CoralArm.INTAKE_STATE.INTAKE : CoralArm.INTAKE_STATE.HOLD);
                    }
            );*/
            inputHandler.listenActionPressAndRelease(
                    "outtakeCoral",
                    (pressed) -> {
                        coralArm.setDesiredIntakeState(pressed ? CoralArm.INTAKE_STATE.OUTTAKE : CoralArm.INTAKE_STATE.REST);
                    }
            );
            inputHandler.listenActionPressAndRelease(
                    "intake/OuttakeAlgae",
                    (pressed) ->{
                        if(pressed){
                            if(algaeCatcher.getDesiredAlgaeCatcherIntakeState() != AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE.HOLD)
                                algaeCatcher.setDesiredState(AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE.INTAKE, AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE.INTAKE);
//                            else
//                                algaeCatcher.setDesiredState(AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE.OUTTAKE, AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE.OUTTAKE);
                        } else{
                            if(algaeCatcher.getDesiredAlgaeCatcherIntakeState() != AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE.HOLD){
                                algaeCatcher.setDesiredState(AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE.STOP, AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE.STOW);
                            }
                        };
                    }
            );
            inputHandler.listenAction(
                    "activationDeepHanger",
                    ActionState.PRESSED,
                    () -> {
                        pneumatic.toggle();
                    }
            );


            /** Operator Commands */

            /*NEW SUBSYSTEM ACTIONS*/
            inputHandler.listenAction(
                    "pivotElevatorAndCoralL1",
                    ActionState.PRESSED,
                    () -> {
                        elevator.setDesiredState(Elevator.ELEVATOR_STATE.L1);
                        coralArm.setDesiredPivotState(CoralArm.PIVOT_STATE.L1);
                    }
            );
            inputHandler.listenAction(
                    "pivotElevatorAndCoralL2",
                    ActionState.PRESSED,
                    () -> {
                        elevator.setDesiredState(Elevator.ELEVATOR_STATE.L2);
                        coralArm.setDesiredPivotState(CoralArm.PIVOT_STATE.L2);
                    }
            );
/*            inputHandler.listenAction(
                    "pivotElevatorAndCoralFeeder",
                    ActionState.PRESSED,
                    () -> {
                        elevator.setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
                        coralArm.setDesiredPivotState(CoralArm.PIVOT_STATE.FEEDER);
                    }
            );
            inputHandler.listenAction(
                    "pivotAlgaeStow",
                    ActionState.PRESSED,
                    () -> {
                        algaeCatcher.setDesiredPivotState(AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE.STOW);
                    }
            );
            inputHandler.listenAction(
                    "pivotAlgaeIntake",
                    ActionState.PRESSED,
                    () -> {
                        algaeCatcher.setDesiredPivotState(AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE.INTAKE);
                    }
            );
            inputHandler.listenAction(
                    "pivotAlgaeOuttake",
                    ActionState.PRESSED,
                    () -> {
                        algaeCatcher.setDesiredPivotState(AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE.OUTTAKE);
                    }
            );*/
            inputHandler.listenAction(
                    "pivotElevatorAndCoralL3",
                    ActionState.PRESSED,
                    () -> {
                        if(CoralArm.robotState.isCoralBeamBreakTriggered){
                            elevator.setDesiredState(Elevator.ELEVATOR_STATE.L3);
                            coralArm.setDesiredPivotState(CoralArm.PIVOT_STATE.L3);
                        }

                    }
            );
            inputHandler.listenAction(
                    "pivotElevatorAndCoralL4",
                    ActionState.PRESSED,
                    () -> {
                        if (CoralArm.robotState.isCoralBeamBreakTriggered) {
                            elevator.setDesiredState(Elevator.ELEVATOR_STATE.L4);
                            coralArm.setDesiredPivotState(CoralArm.PIVOT_STATE.L4);
                        }
                    }
            );



            SmartDashboard.putString("Git Hash", Constants.kGitHash);

        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform when the robot has entered the disabled period
     */
    @Override
    public void disabledInit() {
        try {
            orchestrator.clearThreads();

            enabledLoop.stop();
            // Stop any running autos
            autoModeManager.stopAuto();
            ledManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);
            ledManager.writeToHardware();

            if (autoModeManager.getSelectedAuto() == null) {
                autoModeManager.reset();
            }

//            autopather.autopathMaxCalcMilli = 1000;

            subsystemManager.stop();

            robotState.resetAllStates();
            drive.zeroSensors();

            disabledLoop.start();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform when the robot has entered the autonomous period
     */
    @Override
    public void autonomousInit() {
        disabledLoop.stop();
        orchestrator.stopSong();
        ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);
        ledManager.indicateStatus(LedManager.RobotStatus.AUTONOMOUS);

        drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());

        //TODO add new subsystem inits here
        elevator.setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
        algaeCatcher.setDesiredState(AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE.STOP, AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE.STOW);
        coralArm.setDesiredState(CoralArm.PIVOT_STATE.FEEDER, CoralArm.INTAKE_STATE.REST);

        drive.setControlState(Drive.ControlState.TRAJECTORY_FOLLOWING);
        autoModeManager.startAuto();

//        autopather.autopathMaxCalcMilli = 5;

        autoStart = Timer.getFPGATimestamp();
        enabledLoop.start();
    }


    /**
     * Actions to perform when the robot has entered the teleoperated period
     */
    @Override
    public void teleopInit() {
        try {
            disabledLoop.stop();
            orchestrator.stopSong();
            ledManager.setDefaultStatus(LedManager.RobotStatus.ENABLED);
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);

            infrastructure.startCompressor();

            elevator.setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
            algaeCatcher.setDesiredState(AlgaeCatcher.ALGAE_CATCHER_INTAKE_STATE.STOP, AlgaeCatcher.ALGAE_CATCHER_PIVOT_STATE.STOW);
            coralArm.setDesiredState(CoralArm.PIVOT_STATE.FEEDER, CoralArm.INTAKE_STATE.REST);

//            autopather.autopathMaxCalcMilli = 5;

            teleopStart = Timer.getFPGATimestamp();
            enabledLoop.start();

            drive.resetHeading(
                    robotState.allianceColor == Color.BLUE ?
                            Rotation2d.fromDegrees(0) :
                            Rotation2d.fromDegrees(180)
            );

        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform when the robot has entered the test period
     */
    @Override
    public void testInit() {
        try {
            orchestrator.stopSong();
            double initTime = System.currentTimeMillis();

            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED, LedManager.ControlState.BLINK);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                ledManager.writeToHardware();
            }

            enabledLoop.stop();
            disabledLoop.start();
            drive.zeroSensors();

            ledManager.indicateStatus(LedManager.RobotStatus.DISABLED, LedManager.ControlState.BLINK);

            if (subsystemManager.testSubsystems()) {
                GreenLogger.log("ALL SYSTEMS PASSED");
                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform periodically on the robot when the robot is powered
     */
    @Override
    public void robotPeriodic() {
        try {
            // updating loop timers
            Robot.looperDt = getLastSubsystemLoop();
            Robot.robotDt = getLastRobotLoop();
            loopStart = Timer.getFPGATimestamp();


            if (Constants.kLoggingRobot) {
                looperLogger.append(looperDt);
                robotLoopLogger.append(robotDt);

                if (Constants.kHasCANivore) {
                    canivoreTrafficLogger.append(CANBus.getStatus(Constants.kCANivoreName).BusUtilization);
                }
                lowSpeedTrafficLogger.append(CANBus.getStatus(Constants.kLowSpeedBusName).BusUtilization);
            }

            subsystemManager.outputToSmartDashboard(); // update shuffleboard for subsystem values
            robotState.outputToSmartDashboard(); // update robot state on field for Field2D widget
            autoModeManager.outputToSmartDashboard(); // update shuffleboard selected auto mode
            playlistManager.outputToSmartDashboard(); // update shuffleboard selected song
        } catch (Throwable t) {
            faulted = true;
            GreenLogger.log(t.getMessage());
        }
    }

    /**
     * Actions to perform periodically when the robot is in the disabled period
     */
    @Override
    public void disabledPeriodic() {
//        inputHandler.updateControllerLayout();

        try {
            if (RobotController.getUserButton()) {
                drive.zeroSensors(Constants.kDefaultZeroingPose);
                ledManager.indicateStatus(LedManager.RobotStatus.ZEROING);
            } else {
                // non-camera LEDs will flash red if robot periodic updates fail
                if (faulted) {
                    if (ledManager.getCurrentControlStatus() != LedManager.RobotStatus.ERROR) {
                        ledManager.indicateStatus(LedManager.RobotStatus.ERROR, LedManager.ControlState.BLINK);
                    }
                    ledManager.writeToHardware();
                }
            }

            if (RobotBase.isReal()) {}

            // Periodically check if drivers changed desired auto - if yes, then update the robot's position on the field
            boolean autoChanged = autoModeManager.update();
            if(robotState.dIsAutoDynamic && robotState.dAutoChanged){
                drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());

                ArrayList<Trajectory.State> trajectoryStates = new ArrayList<>();
                var trajectoryActions = robotState.dAutoTrajectoryActions;
                for(int i = 0; i < trajectoryActions.size(); i++) {
                    ArrayList<Trajectory.State> trajectoryActionStates = new ArrayList<>(trajectoryActions.get(i).getTrajectory().getStates());
                    trajectoryStates.addAll(trajectoryActionStates);
                }

                if(trajectoryStates.isEmpty())
                    robotState.field
                            .getObject("Trajectory")
                            .close();
                else
                    robotState.field
                            .getObject("Trajectory")
                            .setTrajectory(
                                    new Trajectory(trajectoryStates)
                            );

                robotState.dAutoChanged = false;
            } else if(!robotState.dIsAutoDynamic) {
                drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());
                robotState.field
                        .getObject("Trajectory")
                        .setTrajectory(
                                autoModeManager.getSelectedAuto().getCurrentTrajectory()
                        );
            }

            if(robotState.dIsAutoDynamic)
                dynamicAutoScript.update();

            if (drive.isDemoMode()) { // Demo-mode
                drive.update();
            }

            playlistManager.update();
            desireToPlaySong = SmartDashboard.getBoolean("PlaySong", false);
            orchestrator.playSong(desireToPlaySong);
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform periodically when the robot is in the autonomous period
     */
    @Override
    public void autonomousPeriodic() {
        robotState.field
                .getObject("Trajectory")
                .setTrajectory(autoModeManager.getSelectedAuto().getCurrentTrajectory());

        if(Constants.kLoggingRobot) {
//            GreenLogger.updatePeriodicLogs();
        }

    }

    /**
     * Actions to perform periodically when the robot is in the teleoperated period
     */
    @Override
    public void teleopPeriodic() {
        try {

            if (Constants.kUseVision) {
                if (robotState.currentCamFind) {
                    orchestrator.updatePoseWithVisionData();
                }
            }

            if(Constants.kLoggingRobot) {
//                GreenLogger.updatePeriodicLogs();
            }

            manualControl();
//            if(robotState.autopathing)
//                autopather.routine();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Sets manual inputs for subsystems like the drivetrain when criteria met
     */
    public void manualControl() {

        inputHandler.update();

        robotState.throttleInput = -inputHandler.getActionAsDouble("throttle");
        robotState.strafeInput = -inputHandler.getActionAsDouble("strafe");
        robotState.rotationInput = -inputHandler.getActionAsDouble("rotation");

        if(robotState.autopathing && (robotState.throttleInput != 0 || robotState.strafeInput != 0) && (double) System.nanoTime() /1000000 - robotState.autopathBeforeTime > robotState.autopathPathCancelBufferMilli){
//            autopather.stop();
        }

        if (robotState.rotatingClosedLoop) {
            drive.rotationPeriodic();
        } else {
            drive.setTeleopInputs(
                    robotState.throttleInput,
                    robotState.strafeInput,
                    robotState.rotationInput
            );
        }
    }

    /**
     * Actions to perform periodically when the robot is in the test period
     */
    @Override
    public void testPeriodic() {}
}
