package com.team1816.core;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.core.auto.AutoModeManager;
import com.team1816.core.configuration.Constants;
import com.team1816.core.configuration.FieldConfig;
import com.team1816.core.states.Orchestrator;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.PlaylistManager;
import com.team1816.lib.auto.Color;
import com.team1816.lib.autopath.PatriotPath;
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
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import com.team1816.season.subsystems.Pneumatic;
import com.team1816.season.subsystems.*;
import com.team1816.season.auto.actions.NamedCommandRegistrar;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
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
    private PatriotPath autopather;
    private Elevator elevator;
    private CoralArm coralArm;
    private Ramp ramp;
    private Pneumatic pneumatic;

    private LedManager ledManager;
    private Camera camera;

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
    private boolean rampZeroed = false;
    private boolean zeroingButtonWasPressed = false;


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
            camera.setDriverMode(0,false);
            robotState = Injector.get(RobotState.class);
            orchestrator = Injector.get(Orchestrator.class);
            infrastructure = Injector.get(Infrastructure.class);
            subsystemManager = Injector.get(SubsystemLooper.class);
            autoModeManager = Injector.get(AutoModeManager.class);
            playlistManager = Injector.get(PlaylistManager.class);
            autopather = Injector.get(PatriotPath.class);
            coralArm = Injector.get(CoralArm.class);
            elevator = Injector.get(Elevator.class);
            ramp = Injector.get(Ramp.class);
            pneumatic = Injector.get(Pneumatic.class);

            /* Register PathPlanner named commands */
            NamedCommandRegistrar.registerCommands();

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

            subsystemManager.setSubsystems(drive, ledManager, camera, coralArm, elevator, ramp, pneumatic);

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

            /**NEW SUBSYSTEM ACTIONS*/
                /** Driver Commands */
                    /**Intake/Driving Inputs*/
            inputHandler.listenAction("patriotPathToCoralSide1",
                    ActionState.PRESSED, () -> {
                Pose2d target = new Pose2d(5.120, 5.460, Rotation2d.fromDegrees(-132.517));
                if (AutoBuilder.shouldFlip()) target = FlippingUtil.flipFieldPose(target);
                
                autopather.start(target);
            });

            inputHandler.listenAction(
                    "zeroPose",
                    ActionState.PRESSED,
                    () ->
                            drive.resetHeading(
                                    Rotation2d.fromDegrees(robotState.fieldToVehicle.getRotation().getDegrees() + 180)
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
            inputHandler.listenActionPressAndRelease(
                    "outtakeCoral",
                    (pressed) -> {
                        if (pressed) {
                            if (robotState.isCoralBeamBreakTriggered)
                                coralArm.setDesiredIntakeState(CoralArm.INTAKE_STATE.OUTTAKE);
                            else
                                ramp.setDesiredState(Ramp.RAMP_STATE.SCORE);
                        } else {
                            orchestrator.setFeederStates(false);
                        }
                    }
            );
            inputHandler.listenAction(
                    "feeder",
                    ActionState.PRESSED,
                    () -> {
                        orchestrator.setFeederStates(
                                robotState.actualElevatorState == Elevator.ELEVATOR_STATE.FEEDER &&
                                        robotState.actualCoralArmPivotState == CoralArm.PIVOT_STATE.FEEDER &&
                                        robotState.actualRampState == Ramp.RAMP_STATE.L234_FEEDER
                        );
                    }
            );
            inputHandler.listenAction(
                    "activationDeepHanger",
                    ActionState.PRESSED,
                    () -> {
                        pneumatic.toggle();
                    }
            );
            inputHandler.listenAction(
                    "rampClimbPosition",
                    ActionState.PRESSED,
                    () -> {
                        if(robotState.actualCoralArmPivotState == CoralArm.PIVOT_STATE.FEEDER && robotState.actualRampState == Ramp.RAMP_STATE.L234_FEEDER){
                            ramp.setDesiredState(Ramp.RAMP_STATE.CLIMB);
                            coralArm.setDesiredState(CoralArm.PIVOT_STATE.CLIMB, CoralArm.INTAKE_STATE.REST);
                        } else {
                            ramp.setDesiredState(Ramp.RAMP_STATE.L234_FEEDER);
                            coralArm.setDesiredState(CoralArm.PIVOT_STATE.FEEDER, CoralArm.INTAKE_STATE.INTAKE);
                        }

                    }
            );

                  /**Position inputs*/

            inputHandler.listenAction(
                    "L2",
                    ActionState.PRESSED,
                    () -> {
                        if (robotState.isCoralBeamBreakTriggered) {
                            elevator.setDesiredState(Elevator.ELEVATOR_STATE.L2_CORAL);
                            coralArm.setDesiredPivotState(CoralArm.PIVOT_STATE.L2_CORAL);
                        }
                        else {
                            if (robotState.actualElevatorState == Elevator.ELEVATOR_STATE.L2_ALGAE) {
                                orchestrator.setFeederStates(false);
                            }
                            else {
                                elevator.setDesiredState(Elevator.ELEVATOR_STATE.L2_ALGAE);
                                coralArm.setDesiredState(CoralArm.PIVOT_STATE.L2_ALGAE, CoralArm.INTAKE_STATE.REMOVE_ALGAE);
                            }
                        }
                    }
            );
            inputHandler.listenAction(
                    "L3",
                    ActionState.PRESSED,
                    () -> {
                        if (robotState.isCoralBeamBreakTriggered) {
                            elevator.setDesiredState(Elevator.ELEVATOR_STATE.L3_CORAL);
                            coralArm.setDesiredPivotState(CoralArm.PIVOT_STATE.L3_CORAL);
                        }
                        else {
                            if (robotState.actualElevatorState == Elevator.ELEVATOR_STATE.L3_ALGAE) {
                                orchestrator.setFeederStates(false);
                            }
                            else {
                                elevator.setDesiredState(Elevator.ELEVATOR_STATE.L3_ALGAE);
                                coralArm.setDesiredState(CoralArm.PIVOT_STATE.L3_ALGAE, CoralArm.INTAKE_STATE.REMOVE_ALGAE);
                            }
                        }
                    }
            );
            inputHandler.listenAction(
                    "L4",
                    ActionState.PRESSED,
                    () -> {
                        elevator.setDesiredState(Elevator.ELEVATOR_STATE.L4);
                        coralArm.setDesiredPivotState(CoralArm.PIVOT_STATE.L4);
                    }
            );

            inputHandler.listenActionPressAndRelease(
                    "robotcentricRight",
                    (pressed) -> {
                        if (pressed) {
                            robotState.robotcentricRequestAmount++;
                            robotState.robotcentricStrafeInput -= robotState.robotcentricInput;
                        } else {
                            robotState.robotcentricRequestAmount--;
                            robotState.robotcentricStrafeInput += robotState.robotcentricInput;
                        }
                    }
            );

            inputHandler.listenActionPressAndRelease(
                    "robotcentricDown",
                    (pressed) -> {
                        if (pressed) {
                            robotState.robotcentricRequestAmount++;
                            robotState.robotcentricThrottleInput -= robotState.robotcentricInput;
                        } else {
                            robotState.robotcentricRequestAmount--;
                            robotState.robotcentricThrottleInput += robotState.robotcentricInput;
                        }
                    }
            );

            inputHandler.listenActionPressAndRelease(
                    "robotcentricLeft",
                    (pressed) -> {
                        if (pressed) {
                            robotState.robotcentricRequestAmount++;
                            robotState.robotcentricStrafeInput += robotState.robotcentricInput;
                        } else {
                            robotState.robotcentricRequestAmount--;
                            robotState.robotcentricStrafeInput -= robotState.robotcentricInput;
                        }
                    }
            );

            inputHandler.listenActionPressAndRelease(
                    "robotcentricUp",
                    (pressed) -> {
                        if (pressed) {
                            robotState.robotcentricRequestAmount++;
                            robotState.robotcentricThrottleInput += robotState.robotcentricInput;
                        } else {
                            robotState.robotcentricRequestAmount--;
                            robotState.robotcentricThrottleInput -= robotState.robotcentricInput;
                        }
                    }
            );


            /** Buttonboard Commands */

            inputHandler.listenAction(
                    "increaseElevatorPivotOffset",
                    ActionState.HELD,
                    () -> {
                        elevator.offsetElevator(0.1);
                    }
            );
            inputHandler.listenAction(
                    "decreaseElevatorPivotOffset",
                    ActionState.HELD,
                    () -> {
                        elevator.offsetElevator(-0.1);
                    }
            );
            inputHandler.listenAction(
                    "increaseCoralArmPivotOffset",
                    ActionState.HELD,
                    () -> {
                        coralArm.offsetCoralPivot(0.1);
                    }
            );
            inputHandler.listenAction(
                    "decreaseCoralArmPivotOffset",
                    ActionState.HELD,
                    () -> {
                        coralArm.offsetCoralPivot(-0.1);
                    }
            );
            inputHandler.listenAction(
                    "increaseRampPivotOffset",
                    ActionState.HELD,
                    () -> {
                        ramp.offsetRamp(0.1);
                    }
            );
            inputHandler.listenAction(
                    "decreaseRampPivotOffset",
                    ActionState.HELD,
                    () -> {
                        ramp.offsetRamp(-0.1);
                    }
            );
            inputHandler.listenAction(
                    "dislodgeCoral",
                    ActionState.PRESSED,
                    () -> {
                        if(robotState.actualRampState == Ramp.RAMP_STATE.DISLODGE_CORAL) {
                            ramp.setDesiredState(Ramp.RAMP_STATE.L234_FEEDER);
                        } else {
                            ramp.setDesiredState(Ramp.RAMP_STATE.DISLODGE_CORAL);
                        }
                    }
            );

            //For testing only
            inputHandler.listenAction(
                    "testingOffsetPose",
                    ActionState.PRESSED,
                    () ->
                            drive.resetEstimatedOdometry(robotState.fieldToVehicle.plus(new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(Math.toRadians(0)))))
            );

            /** Operator Commands */


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

        drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose(robotState.allianceColor));

        ramp.setBraking(true);

        //TODO add new subsystem inits here

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
//            ramp.setDesiredState(Ramp.RAMP_STATE.L234_FEEDER);
            coralArm.setDesiredState(CoralArm.PIVOT_STATE.FEEDER, CoralArm.INTAKE_STATE.INTAKE);

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

            if (Constants.kUseVision) {
                orchestrator.updatePoseWithVisionData();
            }

            if (RobotBase.isSimulation()) {
                FieldConfig.field.getObject("SimActualRobotPosition").setPose(robotState.simActualFieldToVehicle);
            }

            robotState.isElevatorInRange = elevator.isElevatorInRange();
            robotState.isCoralArmPivotInRange = coralArm.isCoralArmPivotInRange();
            SmartDashboard.putBoolean("Elevator in range", robotState.isElevatorInRange);
            SmartDashboard.putBoolean("Coral arm pivot in range", robotState.isCoralArmPivotInRange);
            if (Constants.kLoggingRobot) {
                looperLogger.append(looperDt);
                robotLoopLogger.append(robotDt);

                if (Constants.kHasCANivore) {
                    canivoreTrafficLogger.append(CANBus.getStatus(Constants.kCANivoreName).BusUtilization);
                }
                lowSpeedTrafficLogger.append(CANBus.getStatus(Constants.kLowSpeedBusName).BusUtilization);
            }

//            GreenLogger.log(String.valueOf(robotState.fieldToVehicle));

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

            if (RobotBase.isReal()) { // Ramp zeroing button logic
//                boolean zeroingButtonPressed = ramp.isZeroingButtonPressed();
//                if (zeroingButtonPressed != zeroingButtonWasPressed && zeroingButtonPressed) { // When zeroing button becomes pressed
//                    if (!rampZeroed) { // Zero position the first time the zeroing button is pressed
//                        ramp.zeroSensors();
//                        ledManager.indicateStatus(LedManager.RobotStatus.ZEROING, LedManager.ControlState.BLINK);
//                        ledManager.writeToHardware(); // Need to manually call writeToHardware because it is not normally called when disabled
//                        rampZeroed = true;
//                    }
//                    else { // Set to brake mode the second time the zeroing button is pressed
//                        ramp.setBraking(true);
//                        ledManager.indicateStatus(LedManager.RobotStatus.DISABLED, LedManager.ControlState.SOLID);
//                        ledManager.writeToHardware(); // Need to manually call writeToHardware because it is not normally called when disabled
//                        rampZeroed = false; // Allows for re-zeroing if it was zeroed incorrectly the first time
//                    }
//                }
//                zeroingButtonWasPressed = zeroingButtonPressed;
            }

            // Periodically check if drivers changed desired auto - if yes, then update the robot's position on the field
            if(autoModeManager.update())
                drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose(robotState.allianceColor));

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
//        robotState.field
//                .getObject("Trajectory")
//                .setTrajectory(autoModeManager.getSelectedAuto().getCurrentTrajectory());

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

            if(Constants.kLoggingRobot) {
//                GreenLogger.updatePeriodicLogs();
            }

            manualControl();
            if(robotState.autopathing) {
                autopather.routine();
            }

        } catch (Throwable t) {
            faulted = true;
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
            autopather.stop();
        }

        if (robotState.rotatingClosedLoop) {
            drive.rotationPeriodic();
        } else if (robotState.robotcentricRequestAmount > 0){
            drive.setTeleopInputs(
                    robotState.robotcentricThrottleInput,
                    robotState.robotcentricStrafeInput,
                    robotState.robotcentricRotationInput
            );
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
