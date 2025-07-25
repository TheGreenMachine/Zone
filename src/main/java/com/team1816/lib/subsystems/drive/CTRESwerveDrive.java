package com.team1816.lib.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.auto.Color;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.PIDUtil;
import com.team1816.lib.hardware.components.gyro.Pigeon2Wrapper;
import com.team1816.lib.hardware.factory.MotorFactory;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.team254.DriveSignal;
import com.team1816.core.Robot;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.season.TunerConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * A Class that implements CTRE's SwerveDriveTrain class
 * @see LegacySwerveDrivetrain
 */
@Singleton
public class CTRESwerveDrive extends Drive implements EnhancedSwerveDrive {

    //TODO low priority but someone needs to implement demo mode
    /**
     * Components
     */
    private final LegacySwerveDrivetrain train;
    private LegacySwerveModuleConstants[] swerveModules;
    private TalonFX[] motors;
    private final AudioConfigs audioConfigs = MotorFactory.getAudioConfigs();

    /**
     * Trajectory
     */
    protected List<Rotation2d> headingsList;
    protected int trajectoryIndex = 0;

    /**
     * Control
     */
    private LegacySwerveRequest request;
    private LegacySwerveRequest.FieldCentric fieldCentricRequest;
    private LegacySwerveRequest.RobotCentric robotCentricRequest;
    private LegacySwerveRequest.SwerveDriveBrake brakeRequest;
    private ModuleRequest autoRequest;

    /**
     * Properties
     */
    private SwerveDriveKinematics swerveKinematics;

    // module indices
    public static final int kFrontLeft = 0;
    public static final int kFrontRight = 1;
    public static final int kBackLeft = 2;
    public static final int kBackRight = 3;

    private static final double maxVel12MPS = TunerConstants.kSpeedAt12Volts.magnitude();

    private static final double driveGearRatio = TunerConstants.kDriveGearRatio;

    private double driveScalar;
    private static final double normalDriveScalar = kMaxVelOpenLoopMeters / maxVel12MPS;
    private static final double slowDriveScalar = normalDriveScalar * factory.getConstant(NAME, "slowModeScalar", 0.5);
    private static final double turboDriveScalar = normalDriveScalar * factory.getConstant(NAME, "turboModeScalar", 2);

    private double rotationScalar;
    private static final double slowRotationScalar = factory.getConstant(NAME, "slowModeScalar", 0.5);
    private static final double turboRotationScalar = factory.getConstant(NAME, "turboModeScalar", 2);

    private SPEED_MODE currentSpeedMode;

    private final double driveDeadband = factory.getConstant(NAME, "driveDeadband", 0.1);
    private final double rotationalDeadband = factory.getConstant(NAME, "rotationalDeadband", 0.1) * Math.PI;
    private final double inputDeadband = factory.getConstant(NAME, "inputDeadband", 0.15);

    private final double snapDivisor = factory.getConstant(NAME, "snapDivisor", 40d);

    /**
     * Logging
     */
    private DoubleArrayLogEntry inputLogger; //X, Y, Rotation - raw -1 to 1 from setTeleopInputs
    private StringLogEntry controlRequestLogger;

    private final ArrayList<StatusSignal<Temperature>> motorTemperatures = new ArrayList<>();//Changed from double to temperature

    private ArrayList<DoubleLogEntry> desiredModuleStatesLogger;
    private ArrayList<DoubleLogEntry> actualModuleStatesLogger;

    private StructArrayLogEntry<SwerveModuleState> desiredModuleStructLogger;
    private StructArrayLogEntry<SwerveModuleState> actualModuleStructLogger;



    @Inject
    public CTRESwerveDrive(LedManager lm, Infrastructure inf, RobotState rs) {
        super(lm, inf, rs);

        // Module Characterization
        swerveModules = new LegacySwerveModuleConstants[4];

        swerveModules[kFrontLeft] = factory.getCTRESwerveModule(NAME, "frontLeft");
        swerveModules[kFrontRight] = factory.getCTRESwerveModule(NAME, "frontRight");
        swerveModules[kBackLeft] = factory.getCTRESwerveModule(NAME, "backLeft");
        swerveModules[kBackRight] = factory.getCTRESwerveModule(NAME, "backRight");

        // Drivetrain characterization
        LegacySwerveDrivetrainConstants constants = new LegacySwerveDrivetrainConstants()
                .withCANbusName(TunerConstants.kCANBus.getName())
                .withPigeon2Id(TunerConstants.DrivetrainConstants.Pigeon2Id);

        train = new LegacySwerveDrivetrain(constants, swerveModules);
        train.getDaqThread().setThreadPriority(99); // Making Odometry thread top Priority

        // Control Request Characterization
        fieldCentricRequest = new LegacySwerveRequest.FieldCentric()
                .withDriveRequestType(LegacySwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(LegacySwerveModule.SteerRequestType.MotionMagic)
                .withDeadband(driveDeadband * kMaxVelOpenLoopMeters)
                .withRotationalDeadband(rotationalDeadband * kMaxAngularSpeed);

        robotCentricRequest = new LegacySwerveRequest.RobotCentric()
                .withDriveRequestType(LegacySwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(LegacySwerveModule.SteerRequestType.MotionMagic);

        autoRequest = new ModuleRequest()
                .withModuleStates(new SwerveModuleState[4]);

        brakeRequest = new LegacySwerveRequest.SwerveDriveBrake()
                .withDriveRequestType(LegacySwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(LegacySwerveModule.SteerRequestType.MotionMagic);

        request = fieldCentricRequest;

        if(isDemoMode()) {
            updateSpeedMode(SPEED_MODE.SLOW);
        } else {
            updateSpeedMode(SPEED_MODE.NORMAL);
        }

        // Kinematics characterization & storing motor references
        Translation2d[] moduleLocations = new Translation2d[4];

        motors = new TalonFX[8];
        for (int i = 0; i < 4; i++) {
            moduleLocations[i] = new Translation2d(swerveModules[i].LocationX, swerveModules[i].LocationY);

            motors[i * 2] = train.getModule(i).getDriveMotor();
            motors[i * 2 + 1] = train.getModule(i).getSteerMotor();

            SmartDashboard.putBoolean(toModuleName(i), true);
        }

        swerveKinematics = new SwerveDriveKinematics(moduleLocations);

        // Logging
        if (Constants.kLoggingRobot) {

            inputLogger = new DoubleArrayLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/Inputs");
            controlRequestLogger = new StringLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/ControlRequest");

            desiredModuleStructLogger = StructArrayLogEntry.create(DataLogManager.getLog(), "Drivetrain/Swerve/DesiredStateStruct", SwerveModuleState.struct);
            actualModuleStructLogger = StructArrayLogEntry.create(DataLogManager.getLog(), "Drivetrain/Swerve/ActualStateStruct", SwerveModuleState.struct);

            desiredModuleStatesLogger = new ArrayList<>();
            actualModuleStatesLogger = new ArrayList<>();

            String moduleName;
            for (int i = 0; i < 4; i++) {
                moduleName = toModuleName(i);
                desiredModuleStatesLogger.add(new DoubleLogEntry(DataLogManager.getLog(),
                        "Drivetrain/Swerve/DesiredModuleStates/" + moduleName +"/speedMPS"));
                desiredModuleStatesLogger.add(new DoubleLogEntry(DataLogManager.getLog(),
                        "Drivetrain/Swerve/DesiredModuleStates/" + moduleName +"/angleDegrees"));
                actualModuleStatesLogger.add(new DoubleLogEntry(DataLogManager.getLog(),
                        "Drivetrain/Swerve/ActualModuleStates/" + moduleName +"/speedMPS"));
                actualModuleStatesLogger.add(new DoubleLogEntry(DataLogManager.getLog(),
                        "Drivetrain/Swerve/ActualModuleStates/" + moduleName +"/angleDegrees"));

                motorTemperatures.add(motors[i].getDeviceTemp());
            }

            GreenLogger.addPeriodicLog(new DoubleArrayLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/DesiredSpeeds"), this::getDesiredSpeeds);
            GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/moduleTemps"), motorTemperatures.get(0).asSupplier());
        }

        // Initialise PathPlanner autopath builder configured to CTRE Swerve Drive
        RobotConfig pathRobotConfig = null;
        try {
            pathRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportWarning("Unable to configure PathPlanner!", e.getStackTrace());
        }

        if (pathRobotConfig != null) {
            // https://pathplanner.dev/pplib-getting-started.html#holonomic-swerve
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    () -> chassisSpeed,
                    (ChassisSpeeds speeds, DriveFeedforwards feedforwards) ->
                            setModuleStates(swerveKinematics.toSwerveModuleStates(speeds)),
                            new PPHolonomicDriveController(
                                    new PIDConstants(7.5, 0, 0),
                                    new PIDConstants(7.5, 0, 0)
                            ),
                    pathRobotConfig,
                    () -> robotState.allianceColor == Color.RED
            );

            PathPlannerLogging.setLogActivePathCallback(p -> robotState.field.getObject("trajectory").setPoses(p));
        }
    }

    /**
     * Configuration
     */

    @Override
    public void createPigeon() {
        super.pigeon = new Pigeon2Wrapper(train.getPigeon2());
        GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/Pitch"), pigeon::getPitchValue);
        GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/Roll"), pigeon::getRollValue);
    }

    @Override
    public void configureOrchestra() {
        for (TalonFX motor : motors) {
            motor.getConfigurator().apply(audioConfigs);
            orchestra.addInstrument(motor);
        }
    }

    /**
     * Control
     */

    @Override
    public void setTeleopInputs(double throttle, double strafe, double rotation) {
        double inputNormed = new Translation2d(throttle, strafe).getNorm();
        double deadbander = 1;
        if (inputNormed < inputDeadband) {
            deadbander = 0;
        }

        if (isBraking) {
            request = brakeRequest;
        } else if (robotState.robotcentricRequestAmount > 0){
            request = robotCentricRequest
                    .withVelocityX(throttle * maxVel12MPS * driveScalar)
                    .withVelocityY(strafe * maxVel12MPS * driveScalar)
                    .withRotationalRate(rotation * kMaxAngularSpeed * Math.PI * rotationScalar);
        } else {
            request = fieldCentricRequest
                    .withVelocityX(throttle  * maxVel12MPS * driveScalar * deadbander)
                    .withVelocityY(strafe  * maxVel12MPS * driveScalar * deadbander)
                    .withRotationalRate(rotation * kMaxAngularSpeed * Math.PI * rotationScalar);
        }

        if (Constants.kLoggingDrivetrain) {
            inputLogger.append(new double[] {throttle, strafe, rotation});
        }

        setOpenLoop(null);
    }

    @Override
    public void setOpenLoop(DriveSignal signal) {
        if (controlState != ControlState.OPEN_LOOP) {
            GreenLogger.log("Switching to open loop.");
            controlState = ControlState.OPEN_LOOP;
        }
    }

    @Override
    public synchronized void setBraking(boolean braking) {
        isBraking = braking;

        if (braking) {
            setOpenLoop(DriveSignal.BRAKE);
        }
    }

    @Override
    public synchronized void setSlowMode(boolean slowMode) {
        if (!isDemoMode()) {
            super.setSlowMode(slowMode);
            updateSpeedMode(slowMode ? SPEED_MODE.SLOW : SPEED_MODE.NORMAL);
        }
    }

    @Override
    public synchronized void setTurboMode(boolean turboMode) {
        if (!isDemoMode()) {
            super.setTurboMode(turboMode);
            updateSpeedMode(turboMode ? SPEED_MODE.TURBO : SPEED_MODE.NORMAL);
        }
    }

    @Override
    public void setModuleStates(SwerveModuleState... desiredStates) {
        request = autoRequest.withModuleStates(desiredStates);

        train.setControl(request);
    }

    private void updateSpeedMode(SPEED_MODE desiredSpeedMode) {
        if (desiredSpeedMode != currentSpeedMode) {
            GreenLogger.log("Speed Mode changed from: " + currentSpeedMode + " to: " + desiredSpeedMode);
            currentSpeedMode = desiredSpeedMode;
            driveScalar = currentSpeedMode.driveScalar;
            rotationScalar = currentSpeedMode.rotationScalar;

            fieldCentricRequest
                    .withDeadband(driveDeadband * currentSpeedMode.driveScalar)
                    .withRotationalDeadband(rotationalDeadband * currentSpeedMode.rotationScalar);
        }
    }

    /**
     * Subsystem periodic
     */

    @Override
    public synchronized void readFromHardware() {
        super.readFromHardware();

        if (train.getState().ModuleStates != null) {
            chassisSpeed = swerveKinematics.toChassisSpeeds(train.getState().ModuleStates);
        }

        if (Constants.kLoggingRobot) {
            for (int i = 0; i < 4; i++) {
                motorTemperatures.get(i).refresh();
                if (motorTemperatures.get(i).getValueAsDouble() > 55) { //TODO YAML
                    SmartDashboard.putBoolean(toModuleName(i), false);
                }
            }
        }


        updateRobotState();
    }

    @Override
    public synchronized void writeToHardware() {
        if (controlState == ControlState.OPEN_LOOP) {
            train.setControl(request);
        }
    }

    @Override
    public boolean testSubsystem() {
        //TODO
        return true;
    }

    @Override
    public void stop() {
        train.setControl(new LegacySwerveRequest.FieldCentric());
    }

    /**
     * State updating
     */

    @Override
    public void zeroSensors(Pose2d pose) {
        resetOdometry(pose);
        startingPose = pose;
        chassisSpeed = new ChassisSpeeds();
        isBraking = false;
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        train.seedFieldRelative(pose);
        updateRobotState();
    }

    @Override
    public void resetEstimatedOdometry(Pose2d pose) {
        resetOdometry(pose);
    }

    public void resetHeading(Rotation2d rotation) {
        GreenLogger.log("Resetting Headings!");
        train.setOperatorPerspectiveForward(rotation);
        updateRobotState();
    }

    public void updateOdometryWithVision(Pose2d estimatedPose2D, double timestamp, Matrix<N3, N1> stdDevs) {
        train.addVisionMeasurement(estimatedPose2D, timestamp, stdDevs);
    }

    @Override
    public void updateRobotState() {
        if (RobotBase.isSimulation()) {
            train.updateSimState(Robot.looperDt / 1000, RobotController.getBatteryVoltage());
        }
        robotState.fieldToVehicle = train.getState().Pose;

        var cs = new ChassisSpeeds(
                chassisSpeed.vxMetersPerSecond,
                chassisSpeed.vyMetersPerSecond,
                chassisSpeed.omegaRadiansPerSecond
        );

        robotState.calculatedVehicleAccel =
                new ChassisSpeeds(
                        (cs.vxMetersPerSecond - robotState.deltaVehicle.vxMetersPerSecond) /
                                Robot.looperDt,
                        (cs.vyMetersPerSecond - robotState.deltaVehicle.vyMetersPerSecond) /
                                Robot.looperDt,
                        -9.80
                );
        robotState.deltaVehicle = cs;


        robotState.vehicleToFloorProximityCentimeters = infrastructure.getMaximumProximity();

        if (Constants.kLoggingDrivetrain) {
            double[] desiredSpeeds = getDesiredSpeeds();

            robotState.drivetrainTemp = motorTemperatures.get(0).getValueAsDouble();

            drivetrainPoseLogger.append(robotState.fieldToVehicle);
            drivetrainChassisSpeedsLogger.append(new double[]{robotState.deltaVehicle.vxMetersPerSecond, robotState.deltaVehicle.vyMetersPerSecond, robotState.deltaVehicle.omegaRadiansPerSecond});

            controlRequestLogger.append(request.getClass().getSimpleName());

            var desiredModuleStates = swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                    desiredSpeeds[0],
                    desiredSpeeds[1],
                    desiredSpeeds[2],
                    robotState.fieldToVehicle.getRotation()
            ));
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    desiredModuleStates,
                    maxVel12MPS
            );
            var actualModuleStates = train.getState().ModuleStates;

            for (int i = 0; i < 4; i++) {
                desiredModuleStates[i] = SwerveModuleState.optimize(desiredModuleStates[i], robotState.fieldToVehicle.getRotation());
                desiredModuleStatesLogger.get(i * 2).append(desiredModuleStates[i].speedMetersPerSecond);
                desiredModuleStatesLogger.get(i * 2 + 1).append(desiredModuleStates[i].angle.getDegrees());

                if (actualModuleStates!= null) {
                    actualModuleStatesLogger.get(i * 2).append(actualModuleStates[i].speedMetersPerSecond);
                    actualModuleStatesLogger.get(i * 2 + 1).append(actualModuleStates[i].angle.getDegrees());
                }
            }

            desiredModuleStructLogger.append(desiredModuleStates);
            if (actualModuleStates != null) {
                actualModuleStructLogger.append(actualModuleStates);
            }

            robotState.robotChassis = chassisSpeed;
        }
    }

    /**
     * Trajectory
     */

    /**
     * Starts a trajectory to be followed with headings (rotate while moving)
     *
     * @param trajectory Trajectory
     * @param headings   Headings (for swerve)
     * @see Drive#startTrajectory(Trajectory, List)
     */
    @Override
    public void startTrajectory(Trajectory trajectory, List<Rotation2d> headings) {
        super.startTrajectory(trajectory, headings);
        headingsList = headings;
        trajectoryIndex = 0;
    }

    /**
     * Returns the list of headings for following a path that are transposed onto a path
     *
     * @return trajectoryHeadings
     */
    public Rotation2d getTrajectoryHeadings() {
        if (headingsList == null) {
            return Constants.EmptyRotation2d;
        } else if (trajectoryIndex > headingsList.size() - 1) {
            return headingsList.get(headingsList.size() - 1);
        }
        if (
                getTrajectoryTimestamp() >
                        trajectory.getStates().get(trajectoryIndex).timeSeconds ||
                        trajectoryIndex == 0
        ) trajectoryIndex++;
        if (trajectoryIndex >= headingsList.size()) {
            GreenLogger.log(headingsList.get(headingsList.size() - 1) + " = max");
            return headingsList.get(headingsList.size() - 1);
        }

        double timeBetweenPoints =
                (
                        trajectory.getStates().get(trajectoryIndex).timeSeconds -
                                trajectory.getStates().get(trajectoryIndex - 1).timeSeconds
                );
        Rotation2d heading;
        heading =
                headingsList
                        .get(trajectoryIndex - 1)
                        .interpolate(
                                headingsList.get(trajectoryIndex),
                                getTrajectoryTimestamp() / timeBetweenPoints
                        );

        return heading;
    }


    /**
     * Utility
     */

    public double[] getDesiredSpeeds() {
        if (request instanceof LegacySwerveRequest.FieldCentric) {
            return new double[]{
                    ((LegacySwerveRequest.FieldCentric) request).VelocityX,
                    ((LegacySwerveRequest.FieldCentric) request).VelocityY,
                    ((LegacySwerveRequest.FieldCentric) request).RotationalRate
            };
        } else if (request instanceof ModuleRequest) {
            ChassisSpeeds moduleSpeeds = swerveKinematics.toChassisSpeeds(((ModuleRequest) request).moduleStates);
            return new double[] {
                    moduleSpeeds.vxMetersPerSecond,
                    moduleSpeeds.vyMetersPerSecond,
                    moduleSpeeds.omegaRadiansPerSecond
            };
        } else {
            return new double[3];
        }
    }


    public static String toModuleName(int moduleIndex) {
        return switch (moduleIndex) {
            case 0 -> "frontLeft";
            case 1 -> "frontRight";
            case 2 -> "backLeft";
            case 3 -> "backRight";
            default -> "unknown";
        };
    }

    @Override
    public PIDSlotConfiguration getPIDConfig() {
        PIDSlotConfiguration defaultPIDConfig = PIDUtil.createDefaultPIDSlotConfig();

        return (factory.getSubsystem(NAME).implemented)
                ? factory
                .getSubsystem(NAME)
                .swerveModules.drivePID.getOrDefault("slot0", defaultPIDConfig)
                : defaultPIDConfig;
    }

    /**
     * Returns the associated kinematics of the drivetrain
     *
     * @return swerveKinematics
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveKinematics;
    }

    @Override
    public void rotationPeriodic() {
        if (thetaController.atGoal()) {
            setRotatingClosedLoop(false);
        } else {
            double inputNormed = new Translation2d(robotState.throttleInput, robotState.strafeInput).getNorm();
            double deadbander = 1;
            if (inputNormed < inputDeadband) {
                deadbander = 0;
            }

            double rotationalSpeed = thetaController.calculate(robotState.fieldToVehicle.getRotation().getDegrees(), robotState.targetRotationRadians);

            setModuleStates(swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(robotState.throttleInput  * maxVel12MPS * driveScalar * deadbander, robotState.strafeInput  * maxVel12MPS * driveScalar * deadbander, rotationalSpeed)));

            if (Constants.kLoggingDrivetrain) {
                inputLogger.append(new double[]{robotState.throttleInput, robotState.rotationInput, 0});
            }
        }
    }

    private enum SPEED_MODE {
        NORMAL(normalDriveScalar, 1),
        SLOW(slowDriveScalar, slowRotationScalar),
        TURBO(turboDriveScalar, turboRotationScalar);

        final double driveScalar;
        final double rotationScalar;

        SPEED_MODE(double driveScalar, double rotationScalar) {
            this.driveScalar = driveScalar;
            this.rotationScalar = rotationScalar;
        }
    }
}
