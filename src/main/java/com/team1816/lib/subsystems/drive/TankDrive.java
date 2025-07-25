package com.team1816.lib.subsystems.drive;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.PIDUtil;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.util.EnhancedMotorChecker;
import com.team1816.lib.util.driveUtil.DriveConversions;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.team254.CheesyDriveHelper;
import com.team1816.lib.util.team254.DriveSignal;
import com.team1816.lib.util.team254.SwerveDriveSignal;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import static com.team1816.lib.util.driveUtil.DriveConversions.*;

@Singleton
public class TankDrive extends Drive implements DifferentialDrivetrain {

    /**
     * Components
     */
    private final IGreenMotor leftMain, rightMain;
    private final IGreenMotor leftFollowerA, rightFollowerA, leftFollowerB, rightFollowerB;

    /**
     * Odometry
     */
    private final DifferentialDriveOdometry simActualTankOdometry;
    private final DifferentialDrivePoseEstimator tankEstimator;
    public static final DifferentialDriveKinematics tankKinematics = new DifferentialDriveKinematics(
            kDriveWheelTrackWidthMeters
    );
    private final CheesyDriveHelper driveHelper = new CheesyDriveHelper();
    private final double tickRatioPerLoop = Constants.kLooperDt / .1d; // Convert Ticks/100MS into Ticks/Robot Loop

    /**
     * States
     */
    public double leftPowerDemand, rightPowerDemand; // % Output (-1 to 1) - used in OPEN_LOOP
    public double leftVelDemand, rightVelDemand; // Velocity (Ticks/100MS) - used in TRAJECTORY_FOLLOWING

    private double leftActualDistance = 0, rightActualDistance = 0; // Meters

    private double leftActualVelocity, rightActualVelocity; // Ticks/100MS

    double leftErrorClosedLoop;
    double rightErrorClosedLoop;

    /**
     * Instantiates a swerve drivetrain from base subsystem parameters
     *
     * @param lm  LEDManager
     * @param inf Infrastructure
     * @param rs  RobotState
     * @see Drive#Drive(LedManager, Infrastructure, RobotState)
     */
    @Inject
    public TankDrive(LedManager lm, Infrastructure inf, RobotState rs) {
        super(lm, inf, rs);
        // configure motors
        leftMain = factory.getMotor(NAME, "leftMain");
        leftFollowerA = factory.getFollowerMotor(NAME, "leftFollower", leftMain, false);
        leftFollowerB = factory.getFollowerMotor(NAME, "leftFollowerTwo", leftMain, false);
        rightMain = factory.getMotor(NAME, "rightMain");
        rightFollowerA = factory.getFollowerMotor(NAME, "rightFollower", rightMain, false);
        rightFollowerB = factory.getFollowerMotor(NAME, "rightFollowerTwo", rightMain, false);

        // configure follower motor currentLimits
        var currentLimitConfig = new SupplyCurrentLimitConfiguration(
            true,
            factory.getConstant(NAME, "currentLimit", 40),
            0,
            0
        );
        leftMain.configCurrentLimit(
            currentLimitConfig
        );
        leftFollowerA.configCurrentLimit(
            currentLimitConfig
        );
        leftFollowerB.configCurrentLimit(
            currentLimitConfig
        );
        rightMain.configCurrentLimit(
            currentLimitConfig
        );
        rightFollowerA.configCurrentLimit(
            currentLimitConfig
        );
        rightFollowerB.configCurrentLimit(
            currentLimitConfig
        );

        setOpenLoop(DriveSignal.NEUTRAL);

        simActualTankOdometry =
            new DifferentialDriveOdometry(
                getActualHeading(),
                leftActualDistance,
                rightActualDistance
            );

        tankEstimator = robotState.tankEstimator;

        if (Constants.kLoggingDrivetrain) {
            desStatesLogger = new DoubleArrayLogEntry(DataLogManager.getLog(), "Drivetrain/Tank/DesStates");
            actStatesLogger = new DoubleArrayLogEntry(DataLogManager.getLog(), "Drivetrain/Tank/ActStates");
            gyroPitchLogger = new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain/Tank/Pitch");
            gyroRollLogger = new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain/Tank/Roll");
        }

        // Initialise PathPlanner autopath builder configured to Differential Drive
        // Not enabled. In order to enable, please provide an implementation for the fourth
        // parameter.
        /*
        RobotConfig pathRobotConfig = null;
        try {
            pathRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportWarning("Unable to configure PathPlanner!", e.getStackTrace());
        }

        if (pathRobotConfig != null) {
            // https://pathplanner.dev/pplib-getting-started.html#ltv-differential
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    () -> chassisSpeed,
                    (speeds, feedforwards) -> {
                        // no-op
                    },
                    new PPLTVController(0.02),
                    pathRobotConfig,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                    }
            );
        }

        PathPlannerLogging.setLogActivePathCallback(p -> robotState.field.getObject("trajectory").setPoses(p));
        */
        GreenLogger.log("PathPlanner autos disabled for Tank Drive.");
    }

    /**
     * Read/Write Periodic
     */

    /**
     * Writes outputs / demands to hardware on the drivetrain such as motors and handles the desired state of the left
     * and right sides. Directly writes to the motors.
     *
     * @see IGreenMotor
     */
    @Override
    public synchronized void writeToHardware() {// sets the demands for hardware from the inputs provided
        if (controlState == ControlState.OPEN_LOOP) {
            leftMain.set(
                GreenControlMode.PERCENT_OUTPUT,
                isSlowMode ? (leftPowerDemand * 0.5) : leftPowerDemand
            );
            rightMain.set(
                GreenControlMode.PERCENT_OUTPUT,
                isSlowMode ? (rightPowerDemand * 0.5) : rightPowerDemand
            );
        } else {
            leftMain.set(GreenControlMode.VELOCITY_CONTROL, leftVelDemand);
            rightMain.set(GreenControlMode.VELOCITY_CONTROL, rightVelDemand);
        }
    }

    /**
     * Reads outputs from hardware on the drivetrain such as sensors and handles the actual state the wheels and
     * drivetrain speeds. Used to update odometry and other related data.
     *
     * @see Infrastructure
     * @see RobotState
     */
    @Override
    public synchronized void readFromHardware() {
        super.readFromHardware();
        // update current motor velocities and distance traveled
        leftActualVelocity = leftMain.getSensorVelocity();
        rightActualVelocity = rightMain.getSensorVelocity();
        leftActualDistance += ticksToMeters(leftActualVelocity * tickRatioPerLoop);
        rightActualDistance += ticksToMeters(rightActualVelocity * tickRatioPerLoop);

        // update error (only if in closed loop where knowing it is useful)
        if (controlState == ControlState.TRAJECTORY_FOLLOWING) {
            leftErrorClosedLoop = leftMain.get_ClosedLoopError();
            rightErrorClosedLoop = rightMain.get_ClosedLoopError();
        }

        // update current movement of the whole drivetrain
        chassisSpeed =
            tankKinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(getLeftMPSActual(), getRightMPSActual())
            );

        // update actual heading from gyro (pigeon)
        if (RobotBase.isSimulation()) {
//            simulateGyroOffset();
        }
        actualHeading = Rotation2d.fromDegrees(pigeon.getYawValue());

        simActualTankOdometry.update(actualHeading, leftActualDistance, rightActualDistance);
        tankEstimator.update(actualHeading, leftActualDistance, rightActualDistance);

        if (Constants.kLoggingDrivetrain) {
            ((DoubleArrayLogEntry) desStatesLogger).append(new double[]{leftVelDemand, rightVelDemand});
            ((DoubleArrayLogEntry) actStatesLogger).append(new double[]{leftActualVelocity, rightActualVelocity});
        }

        updateRobotState();
    }

    @Override
    public void configureOrchestra() {

    }

    /** Config */

    /**
     * Zeroes the encoders and odometry based on a certain pose
     *
     * @param pose Pose2d
     * @see Drive#zeroSensors()
     */
    @Override
    public void zeroSensors(Pose2d pose) {
        GreenLogger.log("Zeroing drive sensors using "+pose);

        actualHeading = Rotation2d.fromDegrees(pigeon.getYawValue());
        resetEncoders();
        resetOdometry(pose);
        startingPose = pose;

        chassisSpeed = new ChassisSpeeds();
        isBraking = false;
    }

    /**
     * Stops the drivetrain
     *
     * @see Drive#stop()
     */
    @Override
    public synchronized void stop() {
        setOpenLoop(
            controlState == ControlState.OPEN_LOOP
                ? DriveSignal.NEUTRAL
                : DriveSignal.BRAKE
        );
        setBraking(controlState == ControlState.TRAJECTORY_FOLLOWING);
    }

    /**
     * Resets the encoders to hold the zero value
     *
     * @see this#zeroSensors(Pose2d)
     */
    public synchronized void resetEncoders() {
        leftMain.setSensorPosition(0, 0);
        rightMain.setSensorPosition(0, 0);
        leftActualDistance = 0;
        rightActualDistance = 0;
    }

    /**
     * Resets the odometry to a certain pose
     *
     * @param pose Pose2d
     */
    @Override
    public void resetOdometry(Pose2d pose) {
        tankEstimator.resetPosition(
            getActualHeading(),
            leftActualDistance,
            rightActualDistance,
            pose
        );
        tankEstimator.update(actualHeading, leftActualDistance, rightActualDistance);
        simActualTankOdometry.resetPosition(
                getActualHeading(),
                leftActualDistance,
                rightActualDistance,
                pose
        );
        simActualTankOdometry.update(actualHeading, leftActualDistance, rightActualDistance);
        updateRobotState();
    }

    /**
     * Resets the odometry to a certain pose without resetting simulated "actual" positions
     *
     * @param pose Pose2d
     */
    @Override
    public void resetEstimatedOdometry(Pose2d pose) {
        tankEstimator.resetPosition(
                getActualHeading(),
                leftActualDistance,
                rightActualDistance,
                pose
        );
        tankEstimator.update(actualHeading, leftActualDistance, rightActualDistance);
    }

    public void updateOdometryWithVision(Pose2d estimatedPose2D, double timestamp, Matrix<N3, N1> stdDevs) {
        tankEstimator.addVisionMeasurement(estimatedPose2D, timestamp, stdDevs);
    }

    /**
     * Updates robotState based on values from odometry and sensor readings in readFromHardware
     *
     * @see RobotState
     */
    @Override
    public void updateRobotState() {
        robotState.simActualFieldToVehicle = simActualTankOdometry.getPoseMeters();
        robotState.fieldToVehicle = tankEstimator.getEstimatedPosition();

        var cs = new ChassisSpeeds(
            chassisSpeed.vxMetersPerSecond,
            chassisSpeed.vyMetersPerSecond,
            chassisSpeed.omegaRadiansPerSecond
        );
        robotState.calculatedVehicleAccel =
            new ChassisSpeeds(
                (cs.vxMetersPerSecond - robotState.deltaVehicle.vxMetersPerSecond) /
                    Constants.kLooperDt,
                (cs.vyMetersPerSecond - robotState.deltaVehicle.vyMetersPerSecond) /
                    Constants.kLooperDt,
                cs.omegaRadiansPerSecond - robotState.deltaVehicle.omegaRadiansPerSecond
            );
        robotState.deltaVehicle = cs;

        robotState.vehicleToFloorProximityCentimeters = infrastructure.getMaximumProximity();

        if (Constants.kLoggingDrivetrain) {
            drivetrainPoseLogger.append(robotState.fieldToVehicle);
            drivetrainChassisSpeedsLogger.append(new double[]{robotState.deltaVehicle.vxMetersPerSecond, robotState.deltaVehicle.vyMetersPerSecond, robotState.deltaVehicle.omegaRadiansPerSecond});
            gyroPitchLogger.append(pigeon.getPitchValue());
            gyroRollLogger.append(pigeon.getRollValue());
        }
    }

    /** Open loop control */

    /**
     * Sets open loop percent output commands based on the DriveSignal from setTeleOpInputs()
     *
     * @param signal DriveSignal
     * @see Drive#setOpenLoop(DriveSignal)
     */
    @Override
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (controlState != ControlState.OPEN_LOOP) {
            GreenLogger.log("switching to open loop");
            controlState = ControlState.OPEN_LOOP;
            leftErrorClosedLoop = 0;
            rightErrorClosedLoop = 0;
        }
        leftPowerDemand = signal.getLeft();
        rightPowerDemand = signal.getRight();

        leftVelDemand = leftPowerDemand * DriveConversions.metersPerSecondToTicksPer100ms(kMaxVelOpenLoopMeters);
        rightVelDemand = rightPowerDemand * DriveConversions.metersPerSecondToTicksPer100ms(kMaxVelOpenLoopMeters);
    }

    /**
     * Translates teleoperated inputs into a DriveSignal to be used in setTeleOpInputs()
     *
     * @param forward  forward demand
     * @param strafe   strafe demand
     * @param rotation rotation demand
     * @see this#setOpenLoop(DriveSignal)
     * @see Drive#setTeleopInputs(double, double, double)
     * @see SwerveDriveSignal
     */
    @Override
    public void setTeleopInputs(double forward, double strafe, double rotation) {
        DriveSignal driveSignal = driveHelper.cheesyDrive(
            (isDemoMode ? forward * demoModeMultiplier : forward),
            (isDemoMode ? rotation * demoModeMultiplier : rotation),
            false,
            false
        );

        // To avoid overriding brake command
        if (!isBraking) {
            setOpenLoop(driveSignal);
        }
    }

    /**
     * Adapts a DriveSignal for closed loop PID controlled motion
     *
     * @param signal DriveSignal
     */
    public synchronized void setVelocity(DriveSignal signal) {
        if (controlState == ControlState.OPEN_LOOP) {
            leftMain.selectPIDSlot(0);
            rightMain.selectPIDSlot(0);

            leftMain.config_NeutralDeadband(0.0);
            rightMain.config_NeutralDeadband(0.0);
        }

        leftVelDemand = signal.getLeft();
        rightVelDemand = signal.getRight();
    }

    /**
     * Sub-container of gyroscopic based balancing with manual adjustment factors for a swerve drivetrain
     *
     * @see Drive#autoBalance(ChassisSpeeds)
     */
    @Override
    public void autoBalance(ChassisSpeeds fieldRelativeChassisSpeeds) {
        double pitch = pigeon.getPitchValue();
        double roll = pigeon.getRollValue();
        double throttle = 0;
        double strafe = 0;
        var heading = Constants.EmptyRotation2d;

        double correction = (getInitialYaw() - pigeon.getYawValue()) / 1440;
    }

    /**
     * Utilizes a DriveSignal to adapt Trajectory demands for TRAJECTORY_FOLLOWING and closed loop control
     *
     * @param leftVel  left velocity
     * @param rightVel right velocity
     */
    public void updateTrajectoryVelocities(Double leftVel, Double rightVel) {
        // Velocities are in m/sec comes from trajectory command
        var signal = new DriveSignal(
            metersPerSecondToTicksPer100ms(leftVel),
            metersPerSecondToTicksPer100ms(rightVel)
        );
        setVelocity(signal);
    }

    /**
     * General getters and setters
     */

    /**
     * Sets whether the drivetrain is braking
     *
     * @param braking boolean
     * @see Drive#setBraking(boolean)
     */
    @Override
    public synchronized void setBraking(boolean braking) {
        if (isBraking != braking) {
            GreenLogger.log("braking: " + braking);
            isBraking = braking;

            if (braking) {
                leftMain.set(GreenControlMode.VELOCITY_CONTROL, 0);
                rightMain.set(GreenControlMode.VELOCITY_CONTROL, 0);
            }
            // TODO ensure that changing neutral modes won't backfire while we're using brushless motors
            NeutralMode mode = braking ? NeutralMode.Brake : NeutralMode.Coast;

            rightMain.setNeutralMode(mode);
            rightFollowerA.setNeutralMode(mode);
            rightFollowerB.setNeutralMode(mode);

            leftMain.setNeutralMode(mode);
            leftFollowerA.setNeutralMode(mode);
            leftFollowerB.setNeutralMode(mode);
        }
    }

    /**
     * Returns the actual velocity of the left side in meters per second
     *
     * @return left velocity (m/s)
     * @see com.team1816.lib.util.driveUtil.DriveConversions#ticksPer100MSToMPS(double)
     */
    public double getLeftMPSActual() {
        return ticksPer100MSToMPS(leftActualVelocity);
    }

    /**
     * Returns the actual velocity of the right side in meters per second
     *
     * @return right velocity (m/s)
     * @see com.team1816.lib.util.driveUtil.DriveConversions#ticksPer100MSToMPS(double)
     */
    public double getRightMPSActual() {
        return ticksPer100MSToMPS(rightActualVelocity);
    }

    /**
     * Returns the left velocity demand in ticks per 100ms
     *
     * @return leftVelDemand
     */
    @Override
    public double getLeftVelocityTicksDemand() {
        return leftVelDemand;
    }

    /**
     * Returns the right velocity demand in ticks per 100ms
     *
     * @return rightVelDemand
     */
    @Override
    public double getRightVelocityTicksDemand() {
        return rightVelDemand;
    }

    /**
     * Returns the actual left velocity in ticks per 100ms
     *
     * @return leftVelActual
     * @see IMotorController
     * @see IGreenMotor
     */
    @Override
    public double getLeftVelocityTicksActual() {
        return leftMain.getSensorVelocity();
    }

    /**
     * Returns the actual right velocity in ticks per 100ms
     *
     * @return rightVelActual
     * @see IMotorController
     * @see IGreenMotor
     */
    @Override
    public double getRightVelocityTicksActual() {
        return rightMain.getSensorVelocity();
    }

    /**
     * Returns the total distance (not displacement) traveled by the left side of the drivetrain
     *
     * @return leftActualDistance
     */
    @Override
    public double getLeftDistance() {
        return leftActualDistance;
    }

    /**
     * Returns the total distance (not displacement) traveled by the right side of the drivetrain
     *
     * @return rightActualDistance
     */
    @Override
    public double getRightDistance() {
        return rightActualDistance;
    }

    /**
     * Returns the left side closed loop error (in-built)
     *
     * @return leftErrorClosedLoop
     */
    @Override
    public double getLeftError() {
        return leftErrorClosedLoop;
    }

    /**
     * Returns the right side closed loop error (in-built)
     *
     * @return rightErrorClosedLoop
     */
    @Override
    public double getRightError() {
        return rightErrorClosedLoop;
    }

    /** config and tests */

    /**
     * Tests the drivetrain by seeing if each side can go back and forth
     *
     * @return true if tests passed
     * @see Drive#testSubsystem()
     */
    @Override
    public boolean testSubsystem() {
        boolean leftSide = EnhancedMotorChecker.checkMotor(this, leftMain);
        boolean rightSide = EnhancedMotorChecker.checkMotor(this, rightMain);

        boolean checkPigeon = pigeon == null;

        GreenLogger.log(leftSide && rightSide && checkPigeon);
        if (leftSide && rightSide && checkPigeon) {
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
        } else {
            ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
        }
        return leftSide && rightSide;
    }

    /**
     * Returns the pid configuration of the motors
     *
     * @return PIDSlotConfiguration
     * @see Drive#getPIDConfig()
     */
    @Override
    public PIDSlotConfiguration getPIDConfig() {
        PIDSlotConfiguration defaultPIDConfig = PIDUtil.createDefaultPIDSlotConfig();

        return (factory.getSubsystem(NAME).implemented)
            ? factory.getSubsystem(NAME).pidConfig.getOrDefault("slot0", defaultPIDConfig)
            : defaultPIDConfig;
    }

    /**
     * Returns the associated kinematics with the drivetrain
     *
     * @return tankKinematics
     */
    public DifferentialDriveKinematics getKinematics() {
        return tankKinematics;
    }
}
