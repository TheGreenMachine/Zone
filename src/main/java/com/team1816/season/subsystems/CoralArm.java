package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.GhostMotor;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A subsystem for the coral arm.
 * <p>
 * The coral arm consists of a motor to adjust its pivot angle-- different states for each level,
 * and intake wheels to either intake, outtake, or hold a piece of coral for outtake on the end of
 * the arm.
 */
@Singleton
public class CoralArm extends Subsystem {
    /**
     * Name
     */
    private static final String NAME = "coralArm";

    /**
     * Components
     */
    private final IGreenMotor intakeMotor;
    private final IGreenMotor pivotMotor;

    private final DigitalInput coralSensor;

    /**
     * States
     */
    private PIVOT_STATE desiredPivotState = PIVOT_STATE.FEEDER;
    private INTAKE_STATE desiredIntakeState = INTAKE_STATE.REST;

    private boolean desiredPivotStateChanged = false;
    private boolean desiredIntakeStateChanged = false;

    private double actualPivotVelocity = 0;
    private double actualIntakeVelocity = 0;

    private double pivotCurrentDraw;
    private double intakeCurrentDraw;

    double desiredIntakeVelocity = 0;

    private double desiredPivotPosition = 0;
    private double actualPivotPosition = 0;

    /**
     * Constants
     */
    private final double intakeSpeed = factory.getConstant(NAME, "intakeSpeed", .7);
    private final double outtakeSpeed = factory.getConstant(NAME, "outtakeSpeed", .7);
    private final double holdSpeed = factory.getConstant(NAME, "holdSpeed", 0);

    private final double l1Position = factory.getConstant(NAME, "coralArmL1Position", 1.0);
    private final double l2Position = factory.getConstant(NAME, "coralArmL2Position", 1.0);
    private final double l3Position = factory.getConstant(NAME, "coralArmL3Position", 1.0);
    private final double l4Position = factory.getConstant(NAME, "coralArmL4Position", 1.0);
    private final double feederPosition = factory.getConstant(NAME, "coralArmFeederPosition", 1.0);

    private final double motorRotationsPerDegree = factory.getConstant(NAME, "coralArmMotorRotationsPerDegree", 1);

    /**
     * Logging
     */
    private DoubleLogEntry desiredIntakeVelocityLogger;

    private BooleanLogEntry beamBreakLogger;

    @Inject
    public CoralArm(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);

        intakeMotor = factory.getMotor(NAME, "intakeMotor");
        pivotMotor = factory.getMotor(NAME, "pivotMotor");

        intakeMotor.selectPIDSlot(0);
        pivotMotor.selectPIDSlot(1);

        coralSensor = new DigitalInput((int) factory.getConstant(NAME, "coralSensorChannel", -1));

        super.desStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/desiredPivotPosition");
        super.actStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/actualPivotPosition");

        if (RobotBase.isSimulation()) {
            pivotMotor.setMotionProfileMaxVelocity(12 / 0.05);
            pivotMotor.setMotionProfileMaxAcceleration(12 / 0.08);
            ((GhostMotor) pivotMotor).setMaxVelRotationsPerSec(240);
        }

        if (Constants.kLoggingRobot) {
            GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/Pivot/actualRollerVelocity"), pivotMotor::getSensorVelocity);
            GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/Angle/actualPivotPosition"), intakeMotor::getSensorPosition);
            GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/Pivot/pivotMotorCurrentDraw"), pivotMotor::getMotorOutputCurrent);
            GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/Intake/intakeMotorCurrentDraw"), intakeMotor::getMotorOutputCurrent);

            desiredIntakeVelocityLogger = new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/Intake/desiredIntakeVelocity");
            beamBreakLogger = new BooleanLogEntry(DataLogManager.getLog(), "CoralArm/Intake/beamBreakLogger");
        }
    }

    public void setDesiredPivotState(PIVOT_STATE desiredPivotState) {
        this.desiredPivotState = desiredPivotState;
        desiredPivotStateChanged = true;
    }

    public void setDesiredIntakeState(INTAKE_STATE desiredIntakeState) {
        this.desiredIntakeState = desiredIntakeState;
        desiredIntakeStateChanged = true;
    }

    public void setDesiredState(PIVOT_STATE desiredPivotState, INTAKE_STATE desiredIntakeState) {
        setDesiredPivotState(desiredPivotState);
        setDesiredIntakeState(desiredIntakeState);
    }

    public boolean isBeamBreakTriggered() {
        if(RobotBase.isSimulation())
            return true;

        return !coralSensor.get();
    }

    @Override
    public void readFromHardware() {
        actualPivotPosition = pivotMotor.getSensorPosition();

        actualPivotVelocity = pivotMotor.getSensorVelocity();
        actualIntakeVelocity = intakeMotor.getSensorVelocity();

        pivotCurrentDraw = pivotMotor.getMotorOutputCurrent();
        intakeCurrentDraw = intakeMotor.getMotorOutputCurrent();

        if (robotState.actualIntakeState != desiredIntakeState) {
            robotState.actualIntakeState = desiredIntakeState;
        }

        if (robotState.actualPivotState != desiredPivotState) {
            robotState.actualPivotState = desiredPivotState;
        }

        if (robotState.isBeamBreakTriggered != isBeamBreakTriggered()) {
            robotState.isBeamBreakTriggered = isBeamBreakTriggered();

            if (robotState.isBeamBreakTriggered && desiredIntakeState == INTAKE_STATE.INTAKE) {
                desiredIntakeState = INTAKE_STATE.HOLD;
                desiredIntakeStateChanged = true;
            }
        }

        if (Constants.kLoggingRobot) {
            doubleDesStatesLogger().append(desiredPivotPosition);
            doubleActStatesLogger().append(actualPivotPosition);

            beamBreakLogger.append(isBeamBreakTriggered());
        }
    }

    @Override
    public void writeToHardware() {
        if (desiredIntakeStateChanged) {
            desiredIntakeStateChanged = false;

            switch (desiredIntakeState) {
                case INTAKE -> desiredIntakeVelocity = intakeSpeed;
                case OUTTAKE -> desiredIntakeVelocity = outtakeSpeed;
                case HOLD -> desiredIntakeVelocity = holdSpeed;
                case REST -> desiredIntakeVelocity = 0;
            }

            intakeMotor.set(GreenControlMode.VELOCITY_CONTROL, desiredIntakeVelocity);
            desiredIntakeVelocityLogger.append(desiredIntakeVelocity);
        }

        robotState.coralMechArm.setAngle(robotState.coralMechArmBaseAngle + pivotMotor.getSensorPosition() / motorRotationsPerDegree);

        if (desiredPivotStateChanged) {
            desiredPivotStateChanged = false;

            desiredPivotPosition = getPivotPosition(desiredPivotState);
            pivotMotor.set(GreenControlMode.POSITION_CONTROL, desiredPivotPosition);
        }
    }

    public boolean isCoralArmPivotInRange(){
        return Math.abs(pivotMotor.getSensorPosition() - desiredPivotPosition) < 5;
    }

    public boolean isCoralArmIntakeInRange(){
        return Math.abs(intakeMotor.getSensorVelocity() - desiredIntakeVelocity) < 5;
    }

    @Override
    public void zeroSensors() {
        // no implementation
    }

    @Override
    public void stop() {
        desiredIntakeState = INTAKE_STATE.REST;
    }

    @Override
    public boolean testSubsystem() {
        return true;
    }

    private double getPivotPosition(PIVOT_STATE pivotState) {
        return switch (pivotState) {
            case L1 -> l1Position;
            case L2 -> l2Position;
            case L3 -> l3Position;
            case L4 -> l4Position;
            case FEEDER -> feederPosition;
        };
    }

    private DoubleLogEntry doubleDesStatesLogger() {
        return (DoubleLogEntry) desStatesLogger;
    }

    private DoubleLogEntry doubleActStatesLogger() {
        return (DoubleLogEntry) actStatesLogger;
    }

    public enum PIVOT_STATE {
        L1,
        L2,
        L3,
        L4,
        FEEDER
    }

    public enum INTAKE_STATE {
        INTAKE,
        HOLD,
        OUTTAKE,
        REST
    }
}
