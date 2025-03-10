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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private boolean offsetHasBeenApplied = false;

    private double actualPivotVelocity = 0;
    private double actualIntakeVelocity = 0;

    private double pivotCurrentDraw;
    private double intakeCurrentDraw;

    double desiredIntakePower = 0;

    private double desiredPivotPosition = 0;
    private double actualPivotPosition = 0;

    private double beamBreakLastUntriggeredTimestamp = -100;
    private boolean shouldStopOuttakingSoon = false;

    /**
     * Constants
     */
    private final double intakeSpeed = factory.getConstant(NAME, "intakeSpeed", .7);
    private final double outtakeSpeed = factory.getConstant(NAME, "outtakeSpeed", .7);
    private final double holdSpeed = factory.getConstant(NAME, "holdSpeed", 0);

    private double l1Position = factory.getConstant(NAME, "coralArmL1Position", 1.0);
    private double l2Position = factory.getConstant(NAME, "coralArmL2Position", 1.0);
    private double l3Position = factory.getConstant(NAME, "coralArmL3Position", 1.0);
    private double l4Position = factory.getConstant(NAME, "coralArmL4Position", 1.0);
    private double feederPosition = factory.getConstant(NAME, "coralArmFeederPosition", 1.0);
    private double upPosition = factory.getConstant(NAME, "coralArmUpPosition", 1.0);

    private final double motorRotationsPerDegree = factory.getConstant(NAME, "coralArmMotorRotationsPerDegree", 1);

    private final double stopOuttakingDelay = factory.getConstant(NAME, "stopOuttakingDelay", 2);

    /**
     * Logging
     */
    private DoubleLogEntry desiredIntakePowerLogger;

    private BooleanLogEntry beamBreakLogger;

    @Inject
    public CoralArm(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);

        intakeMotor = factory.getMotor(NAME, "coralArmIntakeMotor");
        pivotMotor = factory.getMotor(NAME, "coralArmPivotMotor");

        pivotMotor.selectPIDSlot(0);

        coralSensor = new DigitalInput((int) factory.getConstant(NAME, "coralSensorChannel", 0));

        super.desStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/desiredPivotPosition");
        super.actStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/actualPivotPosition");

        zeroSensors();

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

            desiredIntakePowerLogger = new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/Intake/desiredIntakePower");
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
//        System.out.println(!coralSensor.get());

        return !coralSensor.get();
    }

    @Override
    public void readFromHardware() {
        actualPivotPosition = pivotMotor.getSensorPosition();

        actualPivotVelocity = pivotMotor.getSensorVelocity();
        actualIntakeVelocity = intakeMotor.getSensorVelocity();

        pivotCurrentDraw = pivotMotor.getMotorOutputCurrent();
        intakeCurrentDraw = intakeMotor.getMotorOutputCurrent();

        //Setting beam break state
        boolean beamBreak = isBeamBreakTriggered();
        if (robotState.isCoralBeamBreakTriggered != beamBreak) {
            robotState.isCoralBeamBreakTriggered = beamBreak;
            if(!robotState.isCoralBeamBreakTriggered) {
                beamBreakLastUntriggeredTimestamp = Timer.getFPGATimestamp();
                shouldStopOuttakingSoon = true;
            }
        }

        //Setting intake motor state
        if(desiredIntakeState == INTAKE_STATE.OUTTAKE && Timer.getFPGATimestamp() >= (beamBreakLastUntriggeredTimestamp + stopOuttakingDelay/*Delay to make sure coral gets fully off*/) && shouldStopOuttakingSoon) {
            desiredIntakeState = INTAKE_STATE.INTAKE;
            shouldStopOuttakingSoon = false;
        }
        if(desiredIntakeState != INTAKE_STATE.OUTTAKE) {
            desiredIntakeState = robotState.isCoralBeamBreakTriggered ? INTAKE_STATE.HOLD : INTAKE_STATE.INTAKE;
        }

        //Setting pivot motor state
        if(robotState.isElevatorInRange) {
            desiredPivotState = switch (robotState.actualElevatorState){
                case L1 -> PIVOT_STATE.L1;
                case L2 -> PIVOT_STATE.L2;
                case L3 -> PIVOT_STATE.L3;
                case L4 -> PIVOT_STATE.L4;
                case FEEDER -> robotState.isCoralBeamBreakTriggered ? PIVOT_STATE.UP : PIVOT_STATE.FEEDER;
            };
        }
        else {
            desiredPivotState = PIVOT_STATE.UP;
        }

        if (robotState.actualCoralArmIntakeState != desiredIntakeState) {
            robotState.actualCoralArmIntakeState = desiredIntakeState;
            desiredIntakeStateChanged = true;
        }

        if (robotState.actualCoralArmPivotState != desiredPivotState) {
            robotState.actualCoralArmPivotState = desiredPivotState;
            desiredPivotStateChanged = true;
        }

        SmartDashboard.putBoolean("CoralArmBeamBreak", beamBreak);

        if (Constants.kLoggingRobot) {
            doubleDesStatesLogger().append(desiredPivotPosition);
            doubleActStatesLogger().append(actualPivotPosition);

            beamBreakLogger.append(beamBreak);
        }
    }

    @Override
    public void writeToHardware() {
        if (desiredIntakeStateChanged) {
            desiredIntakeStateChanged = false;

            switch (desiredIntakeState) {
                case INTAKE -> desiredIntakePower = intakeSpeed;
                case OUTTAKE -> desiredIntakePower = outtakeSpeed;
                case HOLD -> desiredIntakePower = holdSpeed;
                case REST -> desiredIntakePower = 0;
            }

            intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, desiredIntakePower);
            desiredIntakePowerLogger.append(desiredIntakePower);
        }

        robotState.coralMechArm.setAngle(robotState.coralMechArmBaseAngle + pivotMotor.getSensorPosition() / motorRotationsPerDegree);

        if (desiredPivotStateChanged || offsetHasBeenApplied) {
            desiredPivotStateChanged = false;
            offsetHasBeenApplied = false;

            desiredPivotPosition = getPivotPosition(desiredPivotState);
            pivotMotor.set(GreenControlMode.MOTION_MAGIC_EXPO, MathUtil.clamp(desiredPivotPosition, 1, 36));
        }
//        GreenLogger.log("Coral arm intake state: "+desiredIntakeState+" Intake power: "+desiredIntakePower+" Pivot state: "+desiredPivotState+" Pivot position: "+desiredPivotPosition);
    }

    public void offsetCoralPivot(double offsetAmount){
            switch (desiredPivotState) {
                case L1 -> l1Position += offsetAmount;
                case L2 -> l2Position += offsetAmount;
                case L3 -> l3Position += offsetAmount;
                case L4 -> l4Position += offsetAmount;
                case FEEDER -> feederPosition += offsetAmount;
                case UP -> upPosition += offsetAmount;
        }
        offsetHasBeenApplied = true;
        GreenLogger.log("Coral arm " + desiredPivotState + " pivot position set to " + getPivotPosition(desiredPivotState));
    }

    public boolean isCoralArmPivotInRange(){
        return Math.abs(pivotMotor.getSensorPosition() - desiredPivotPosition) < 2;
    }

    public boolean isCoralArmIntakeInRange(){
        return Math.abs(intakeMotor.getMotorOutputPercent() - desiredIntakePower) < 5;
    }

    @Override
    public void zeroSensors() {
        pivotMotor.setSensorPosition(0);
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
            case UP -> upPosition;
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
        FEEDER,
        UP
    }

    public enum INTAKE_STATE {
        INTAKE,
        HOLD,
        OUTTAKE,
        REST
    }
}
