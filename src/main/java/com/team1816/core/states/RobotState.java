package com.team1816.core.states;

import com.google.inject.Singleton;
import com.team1816.core.configuration.Constants;
import com.team1816.core.configuration.FieldConfig;
import com.team1816.lib.auto.Color;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.subsystems.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import org.photonvision.EstimatedRobotPose;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is responsible for logging the robot's actual states and estimated states.
 * Including superstructure and subsystem states.
 */

@Singleton
public class RobotState {

    /**
     * Odometry and field characterization
     */
    public final Field2d field = new Field2d();
    public Color allianceColor = Color.BLUE;
    public Pose2d fieldToVehicle = Constants.EmptyPose2d;
    public Pose2d driverRelativeFieldToVehicle = Constants.EmptyPose2d;
    public ChassisSpeeds deltaVehicle = new ChassisSpeeds(); // velocities of vehicle
    public ChassisSpeeds calculatedVehicleAccel = new ChassisSpeeds(); // calculated acceleration of vehicle
    public Double[] triAxialAcceleration = new Double[]{0d, 0d, 0d};
    public boolean isPoseUpdated = true;
    public double vehicleToFloorProximityCentimeters = 0;
    public double drivetrainTemp = 0;
    public SwerveDrivePoseEstimator swerveEstimator =
            new SwerveDrivePoseEstimator(
                    SwerveDrive.swerveKinematics,
                    Constants.EmptyRotation2d,
                    new SwerveModulePosition[]{
                            new SwerveModulePosition(),
                            new SwerveModulePosition(),
                            new SwerveModulePosition(),
                            new SwerveModulePosition()
                    },
                    new Pose2d() //TODO figure out what to initialize this to
            );

    /**
     * Current Drive inputs and states
     */
    public double throttleInput = 0;
    public double strafeInput = 0;
    public double rotationInput = 0;
    public int robotcentricRequestAmount = 4; //this is here because robot on startup will activate all not pressed commands, so this counters it
    public double robotcentricThrottleInput = 0;
    public double robotcentricStrafeInput = 0;
    public double robotcentricRotationInput = 0;
    public double robotcentricInput = 0.3; //0 to 1

    /**
     * Rotating closed loop
     */

    public boolean rotatingClosedLoop = false;
    public double targetRotationRadians = 0;

    /**
     * Orchestrator states
     */

    //TODO add new subystem states here
    public CoralArm.INTAKE_STATE actualCoralArmIntakeState = CoralArm.INTAKE_STATE.INTAKE;
    public CoralArm.PIVOT_STATE actualCoralArmPivotState = CoralArm.PIVOT_STATE.FEEDER;
    public Elevator.ELEVATOR_STATE actualElevatorState = Elevator.ELEVATOR_STATE.FEEDER;
    public Ramp.RAMP_STATE actualRampState = Ramp.RAMP_STATE.L234_FEEDER;
    public Pneumatic.PNEUMATIC_STATE actualPneumaticState = Pneumatic.PNEUMATIC_STATE.OFF;

    public boolean isCoralBeamBreakTriggered = false;

    public boolean isElevatorInRange = false;
    public boolean isCoralArmPivotInRange = false;

    public VisionPoint superlativeTarget = new VisionPoint();
    public List<VisionPoint> visibleTargets = new ArrayList<>();

    public final Mechanism2d rampMech2d = new Mechanism2d(2, 2);
    public final MechanismRoot2d rampMech2dRoot = rampMech2d.getRoot("root", 1, 1);

    public final double rampMechArmBaseAngle = 80;
    public final MechanismLigament2d rampMechArm = rampMech2dRoot.append(new MechanismLigament2d("rampArm1", 1, rampMechArmBaseAngle));
    public final MechanismLigament2d rampMechArmFunnelSide = rampMech2dRoot.append(new MechanismLigament2d("rampArm2", 1, rampMechArmBaseAngle + 90));


    public final Mechanism2d elevatorAndCoralArmMech2d = new Mechanism2d(4, 4);
    public final MechanismRoot2d elevatorAndCoralArmMech2dRoot = elevatorAndCoralArmMech2d.getRoot("root", 2, 0);

    public final MechanismLigament2d elevatorMechArm = elevatorAndCoralArmMech2dRoot.append(new MechanismLigament2d("stand", 1, 90));
    public final double coralMechArmBaseAngle = 300;
    public final MechanismLigament2d coralMechArm = elevatorMechArm.append(new MechanismLigament2d("pivot", .7, coralMechArmBaseAngle));

    /**
     * Autopathing state
     */
    public boolean autopathing = false;
    public double robotVelocity = 0;
    public double autopathBeforeTime = 0;
    public double autopathPathCancelBufferMilli = 500;
    public ChassisSpeeds robotChassis = new ChassisSpeeds();

    /**
     * Pigeon state
     */

    public double[] gyroPos = new double[3];

    /**
     * Initializes RobotState and field
     */
    public RobotState() {
        resetPosition();
        FieldConfig.setupField(field);
    }

    /**
     * Vision Pose Stuff
     */
    public double lastEstTimestamp = 0;
    public final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public EstimatedRobotPose currentVisionEstimatedPose;
    public boolean currentCamFind;

    /**
     * Resets drivetrain position to a specified pose of drivetrain
     *
     * @param initial_field_to_vehicle
     */
    public synchronized void resetPosition(Pose2d initial_field_to_vehicle) {
        fieldToVehicle = initial_field_to_vehicle;
    }

    /**
     * Resets the drivetrain to its default "zero" pose
     *
     * @see Constants
     */
    public synchronized void resetPosition() {
        resetPosition(Constants.kDefaultZeroingPose);
    }

    /**
     * Resets all values stored in RobotState
     */
    public synchronized void resetAllStates() {
        deltaVehicle = new ChassisSpeeds();
        calculatedVehicleAccel = new ChassisSpeeds();
        triAxialAcceleration = new Double[]{0d, 0d, 0d};

        // TODO: Insert any subsystem state set up here.
        actualElevatorState = Elevator.ELEVATOR_STATE.FEEDER;
        actualRampState = Ramp.RAMP_STATE.L234_FEEDER;
        actualPneumaticState = Pneumatic.PNEUMATIC_STATE.OFF;

        isPoseUpdated = true;
        superlativeTarget = new VisionPoint();
        visibleTargets = new ArrayList<>();
        drivetrainTemp = 0;
        vehicleToFloorProximityCentimeters = 0;
    }

    /**
     * Returns rotation of the camera with respect to the field
     *
     * @return Rotation2d
     * @see Orchestrator#calculateSingleTargetTranslation(VisionPoint) ()
     */
    public Rotation2d getLatestFieldToCamera() {
        return fieldToVehicle.getRotation().plus(Constants.kCameraMountingOffset.getRotation());
    }

    /**
     * Locks robot rotation to a specific angle, then terminates rotation once angle is reached
     *
     * @param targetRotationRadians
     * @return
     */
    public boolean setRobotRotatingClosedLoop(double targetRotationRadians){
        this.targetRotationRadians = targetRotationRadians;
        rotatingClosedLoop = true;

        return fieldToVehicle.getRotation().getRadians() == targetRotationRadians;
    }

    /**
     * Returns the estimated / calculated acceleration of the robot based on sensor readings
     *
     * @return ChassisSpeeds
     */
    public synchronized ChassisSpeeds getCalculatedAccel() {
        return calculatedVehicleAccel;
    }

    /**
     * Outputs real-time telemetry data to Shuffleboard / SmartDashboard
     */
    public synchronized void outputToSmartDashboard() {
        field.setRobotPose(fieldToVehicle);

        SmartDashboard.putData("Elevator+CoralArm", elevatorAndCoralArmMech2d);
        SmartDashboard.putData("Ramp", rampMech2d);
//        System.out.println(fieldToVehicle);

        if (RobotBase.isSimulation()) {
            // TODO: Display any stats here

            // e.g.
            SmartDashboard.putNumber(
                    "Path_to_Subsystem/Value",
                    02390293.23
            );
        }
    }
}
