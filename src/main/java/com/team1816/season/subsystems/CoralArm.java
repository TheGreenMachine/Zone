package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

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

    private SendableChooser<Boolean> simBeamBreakChooser = new SendableChooser<>();

    /**
     * Constants
     */
    private final double intakeSpeed = factory.getConstant(NAME, "intakeSpeed", .7);
    private final double outtakeSpeed = factory.getConstant(NAME, "outtakeSpeed", .7);
    private final double holdSpeed = factory.getConstant(NAME, "holdSpeed", 0);
    private final double removeAlgaeSpeed = factory.getConstant(NAME, "removeAlgaeSpeed", .7);

    private double l1Position = factory.getConstant(NAME, "coralArmL1Position", 1.0);
    private double l2CoralPosition = factory.getConstant(NAME, "coralArmL2CoralPosition", 1.0);
    private double l3CoralPosition = factory.getConstant(NAME, "coralArmL3CoralPosition", 1.0);
    private double l4Position = factory.getConstant(NAME, "coralArmL4Position", 1.0);
    private double feederPosition = factory.getConstant(NAME, "coralArmFeederPosition", 1.0);
    private double upPosition = factory.getConstant(NAME, "coralArmUpPosition", 1.0);
    private double l2AlgaePosition = factory.getConstant(NAME, "coralArmL2AlgaePosition", 1.0);
    private double l3AlgaePosition = factory.getConstant(NAME, "coralArmL3AlgaePosition", 1.0);
    private double climbPosition = factory.getConstant(NAME, "coralArmClimbPosition", 1.0);

    private final double motorRotationsPerDegree = factory.getConstant(NAME, "coralArmMotorRotationsPerDegree", 1);

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

        simBeamBreakChooser.addOption("Triggered", true);
        simBeamBreakChooser.addOption("Not triggered", false);
        simBeamBreakChooser.setDefaultOption("Triggered", true);
        SmartDashboard.putData("Sim coral beam break", simBeamBreakChooser);

        super.desStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/desiredPivotPosition");
        super.actStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "CoralArm/actualPivotPosition");

        zeroSensors();

        if (RobotBase.isSimulation()) {
            pivotMotor.setMotionProfileMaxVelocity(12 / 0.05);
            pivotMotor.setMotionProfileMaxAcceleration(12 / 0.08);
            ((GhostMotor) pivotMotor).setMaxVelRotationsPerSec(34);
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
            return simBeamBreakChooser.getSelected();
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
            if(beamBreak && robotState.actualCoralArmIntakeState == INTAKE_STATE.INTAKE)
                desiredIntakeState = INTAKE_STATE.HOLD;
            else if(!beamBreak && robotState.actualCoralArmIntakeState == INTAKE_STATE.HOLD)
                desiredIntakeState = INTAKE_STATE.INTAKE;
            else if(!beamBreak && robotState.actualCoralArmIntakeState == INTAKE_STATE.REST)
                desiredIntakeState = INTAKE_STATE.INTAKE;

            robotState.isCoralBeamBreakTriggered = beamBreak;
        }

        //Setting intake motor state
        if(robotState.actualRampState == Ramp.RAMP_STATE.L1_FEEDER)
            desiredPivotState = PIVOT_STATE.FEEDER;

        if(desiredPivotState != PIVOT_STATE.CLIMB && robotState.actualCoralArmIntakeState != CoralArm.INTAKE_STATE.OUTTAKE && !robotState.isCoralBeamBreakTriggered && desiredPivotState != PIVOT_STATE.L2_ALGAE && desiredPivotState != PIVOT_STATE.L3_ALGAE)
            desiredPivotState = PIVOT_STATE.FEEDER;


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
                case REMOVE_ALGAE -> desiredIntakePower = removeAlgaeSpeed;
            }

            SmartDashboard.putString("Coral arm desired intake state", String.valueOf(desiredIntakeState));
            SmartDashboard.putNumber("Coral arm desired intake power", desiredIntakePower);

            intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, desiredIntakePower);
            desiredIntakePowerLogger.append(desiredIntakePower);
        }

        robotState.coralMechArm.setAngle(robotState.coralMechArmBaseAngle + pivotMotor.getSensorPosition() / motorRotationsPerDegree);

        if (desiredPivotStateChanged || offsetHasBeenApplied) {
            desiredPivotStateChanged = false;
            offsetHasBeenApplied = false;

            desiredPivotPosition = getPivotPosition(desiredPivotState);
            if (robotState.actualRampState == Ramp.RAMP_STATE.L1_FEEDER)
                desiredPivotStateChanged = true;
            else
                pivotMotor.set(GreenControlMode.MOTION_MAGIC_EXPO, MathUtil.clamp(desiredPivotPosition, -44, -4));
            SmartDashboard.putString("Coral arm desired pivot state", String.valueOf(desiredPivotState));
            SmartDashboard.putNumber("Coral arm desired pivot position", desiredPivotPosition);}
//        GreenLogger.log("Coral arm intake state: "+desiredIntakeState+" Intake power: "+desiredIntakePower+" Pivot state: "+desiredPivotState+" Pivot position: "+desiredPivotPosition);
    }

    public void offsetCoralPivot(double offsetAmount){
            switch (desiredPivotState) {
                case L1 -> l1Position += offsetAmount;
                case L2_CORAL -> l2CoralPosition += offsetAmount;
                case L3_CORAL -> l3CoralPosition += offsetAmount;
                case L4 -> l4Position += offsetAmount;
                case FEEDER -> feederPosition += offsetAmount;
                case UP -> upPosition += offsetAmount;
                case L2_ALGAE -> l2AlgaePosition += offsetAmount;
                case L3_ALGAE -> l3AlgaePosition += offsetAmount;
                case CLIMB -> climbPosition += offsetAmount;
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

    /**
     * Registers commands for:
     * <ul>
     *     <li>coralArm intake intake</li>
     *     <li>coralArm intake outtake</li>
     *     <li>coralArm intake hold</li>
     *     <li>coralArm intake rest</li>
     *     <br>
     *     <li>coralArm pivot l1</li>
     *     <li>coralArm pivot l2</li>
     *     <li>coralArm pivot l3</li>
     *     <li>coralArm pivot l4</li>
     *     <li>coralArm pivot feeder</li>
     *     <li>coralArm pivot up</li>
     * </ul>
     */
    @Override
    public void implementNamedCommands() {
        for (INTAKE_STATE intakeState : INTAKE_STATE.values()) {
            NamedCommands.registerCommand(NAME + " " + intakeState.toString().toLowerCase(),
                    Commands.runOnce(() -> setDesiredIntakeState(intakeState)).alongWith(Commands.waitUntil(this::isCoralArmIntakeInRange)));
        }

        for (PIVOT_STATE pivotState : PIVOT_STATE.values()) {
            NamedCommands.registerCommand(NAME + " " + pivotState.toString().toLowerCase(),
                    Commands.runOnce(() -> setDesiredPivotState(pivotState)).alongWith(Commands.waitUntil(this::isCoralArmPivotInRange)));
        }
    }

    @Override
    public boolean testSubsystem() {
        return true;
    }

    private double getPivotPosition(PIVOT_STATE pivotState) {
        return switch (pivotState) {
            case L1 -> l1Position;
            case L2_CORAL -> l2CoralPosition;
            case L3_CORAL -> l3CoralPosition;
            case L4 -> l4Position;
            case FEEDER -> feederPosition;
            case UP -> upPosition;
            case L2_ALGAE -> l2AlgaePosition;
            case L3_ALGAE -> l3AlgaePosition;
            case CLIMB -> climbPosition;
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
        L2_CORAL,
        L3_CORAL,
        L4,
        FEEDER,
        UP,
        L2_ALGAE,
        L3_ALGAE,
        CLIMB
    }

    public enum INTAKE_STATE {
        INTAKE,
        HOLD,
        OUTTAKE,
        REST,
        REMOVE_ALGAE
    }
}
