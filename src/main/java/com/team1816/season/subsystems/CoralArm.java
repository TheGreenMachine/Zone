package com.team1816.season.subsystems;

import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * A subsystem for the coral arm.
 * <p>
 * The coral arm consists of a motor to adjust its pivot angle-- different states for each level,
 * and intake wheels to either intake, outtake, or hold a piece of coral for outtake on the end of
 * the arm.
 */
public class CoralArm extends Subsystem {
    /**
     * Name
     */
    private static final String NAME = "coral_arm";

    /**
     * Components
     */
    private final IGreenMotor intakeMotor;
    private final IGreenMotor pivotMotor;

    private final DigitalInput coralSensor;

    /**
     * States
     */
    private PIVOT_STATE desiredPivotState;
    private INTAKE_STATE desiredIntakeState;

    private boolean desiredPivotStateChanged;
    private boolean desiredIntakeStateChanged;

    private double actualPivotVelocity;
    private double actualIntakeVelocity;

    private double pivotCurrentDraw;
    private double intakeCurrentDraw;

    private double desiredPivotPosition = 0;
    private double actualPivotPosition = 0;

    /**
     * Constants
     */
    private final double intakeSpeed = factory.getConstant(NAME, "intakeSpeed", .7);
    private final double outtakeSpeed = factory.getConstant(NAME, "outtakeSpeed", .7);

    private final double l1Position = factory.getConstant(NAME, "coralArmL1Position", 1.0);
    private final double l2Position = factory.getConstant(NAME, "coralArmL2Position", 1.0);
    private final double l3Position = factory.getConstant(NAME, "coralArmL3Position", 1.0);
    private final double l4Position = factory.getConstant(NAME, "coralArmL4Position", 1.0);

    /**
     * Logging
     */
    private DoubleLogEntry desiredIntakeVelocityLogger;

    private BooleanLogEntry beamBreakLogger;


    public CoralArm(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);

        intakeMotor = factory.getMotor(NAME, "coralIntakeMotor");
        pivotMotor = factory.getMotor(NAME, "coralPivotMotor");

        intakeMotor.selectPIDSlot(1);
        pivotMotor.selectPIDSlot(2);

        coralSensor = new DigitalInput((int) factory.getConstant(NAME, "coralSensorChannel", -1));

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

    public boolean isBeamBreakTriggered() {
        return !coralSensor.get();
    }

    @Override
    public void readFromHardware() {
        actualPivotPosition = pivotMotor.getSensorPosition();

        actualPivotVelocity = pivotMotor.getSensorVelocity();
        actualIntakeVelocity = intakeMotor.getSensorVelocity();

        pivotCurrentDraw = pivotMotor.getMotorOutputCurrent();
        intakeCurrentDraw = intakeMotor.getMotorOutputCurrent();

        if (robotState.actualPivotState != desiredPivotState) {
            robotState.actualPivotState = desiredPivotState;
        }

        if (robotState.isBeamBreakTriggered != isBeamBreakTriggered()) {
            robotState.isBeamBreakTriggered = isBeamBreakTriggered();
            desiredIntakeStateChanged = true;
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
            double desiredIntakeVelocity = 0;

            switch (desiredIntakeState) {
                case INTAKE -> desiredIntakeVelocity = intakeSpeed;
                case OUTTAKE -> desiredIntakeVelocity = outtakeSpeed;
                case HOLD -> desiredIntakeVelocity = 0;
            }

            intakeMotor.set(GreenControlMode.VELOCITY_CONTROL, desiredIntakeVelocity);
            desiredIntakeVelocityLogger.append(desiredIntakeVelocity);
        }

        if (desiredPivotStateChanged) {
            desiredPivotStateChanged = false;

            desiredPivotPosition = getPivotPosition(desiredPivotState);
            pivotMotor.set(GreenControlMode.MOTION_MAGIC_EXPO, MathUtil.clamp(desiredPivotPosition, 1.5, 35));
        }
    }

    @Override
    public void zeroSensors() {
        // no implementation
    }

    @Override
    public void stop() {
        desiredIntakeState = INTAKE_STATE.HOLD;
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
        };
    }

    private DoubleLogEntry doubleDesStatesLogger() {
        return (DoubleLogEntry) desStatesLogger;
    }

    private DoubleLogEntry doubleActStatesLogger() {
        return (DoubleLogEntry) actStatesLogger;
    }

    public enum PIVOT_STATE {
        L1, L2, L3, L4
    }

    public enum INTAKE_STATE {
        INTAKE,
        HOLD,
        OUTTAKE
    }
}
