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
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Singleton
public class AlgaeCatcher extends Subsystem {
    /**
     * Name
     */
    private static final String NAME = "algaeCatcher";

    /**
     * Components
     */
    private final IGreenMotor intakeMotor;
    private final IGreenMotor pivotMotor;

    private final DigitalInput algaeSensor;

    /**
     * Constants
     */
    private static final double stowPosition = factory.getConstant(NAME,"stowPosition",1.0);
    private static final double intakePosition = factory.getConstant(NAME,"intakePosition",1.0);
    private static final double holdPosition = factory.getConstant(NAME,"holdPosition",1.0);
    private static final double outtakePosition = factory.getConstant(NAME,"outtakePosition",1.0);
    private static final double removeAlgaePosition = factory.getConstant(NAME,"removeAlgaePosition",1.0);

    private final double algaeMotorRotationsPerDegree = factory.getConstant(NAME, "algaeMotorRotationsPerDegree", 1);

    /**
     * Properties
     */
    public final double algaeCollectSpeed;
    public final double algaeHoldSpeed;
    public final double algaeReleaseSpeed;

    /**
     * Logging
     */
    private DoubleLogEntry algaeCatcherCurrentDrawLogger;

    /**
     * States
     */
    private ALGAE_CATCHER_INTAKE_STATE desiredIntakeState = ALGAE_CATCHER_INTAKE_STATE.STOP;
    private ALGAE_CATCHER_PIVOT_STATE desiredPivotState = ALGAE_CATCHER_PIVOT_STATE.STOW;
    private double actualAlgaeCatcherVelocity = 0;
    private double desiredAlgaeCatcherPower = 0;
    private double algaeCatcherCurrentDraw = 0;
    private boolean desiredIntakeStateChanged = false;
    private boolean desiredPivotStateChanged = false;
    private double desiredPosition = 0;
    private double actualPosition = 0;
    private double actualPositionDegrees = 0;

    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    @Inject
    public AlgaeCatcher(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        intakeMotor = factory.getMotor(NAME, "algaeCatcherIntakeMotor");
        pivotMotor = factory.getMotor(NAME, "algaeCatcherPositionMotor");


        algaeSensor = new DigitalInput((int) factory.getConstant(NAME, "algaeSensorChannel", 1));

        algaeCollectSpeed = factory.getConstant(NAME, "algaeCollectSpeed", .3);
        algaeHoldSpeed = factory.getConstant(NAME, "algaeHoldSpeed", 0);
        algaeReleaseSpeed = factory.getConstant(NAME, "algaeReleaseSpeed", -.3);

        SmartDashboard.putBoolean("AlgaeCollector", intakeMotor.getMotorTemperature() < 55);

        pivotMotor.selectPIDSlot(0);

        zeroSensors();

        if (RobotBase.isSimulation()) {
            pivotMotor.setMotionProfileMaxVelocity(12 / 0.05);
            pivotMotor.setMotionProfileMaxAcceleration(12 / 0.08);
            ((GhostMotor) pivotMotor).setMaxVelRotationsPerSec(240);
        }

        if (Constants.kLoggingRobot) {
            algaeCatcherCurrentDrawLogger = new DoubleLogEntry(DataLogManager.getLog(), "AlgaeCatcher/Velocity/desiredAlgaeCurrent");
            desStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "Collector/desiredAlgaeCatcherPower");
            GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "Collector/actualAlgaeCatcherPower"), intakeMotor::getMotorOutputPercent);
            GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "Collector/algaeCatcherCurrentDraw"), intakeMotor::getMotorOutputCurrent);
        }
    }

    /**
     * Sets the desired state of the collector
     *
     * @param desiredIntakeState COLLECTOR_STATE
     */
    public void setDesiredState(ALGAE_CATCHER_INTAKE_STATE desiredIntakeState, ALGAE_CATCHER_PIVOT_STATE desiredPositionState) {
        setDesiredIntakeState(desiredIntakeState);
        setDesiredPivotState(desiredPositionState);
    }

    /**
     * Sets the desired state of the position
     *
     * @param desiredPivotState POSITION_STATE
     */
    public void setDesiredPivotState(ALGAE_CATCHER_PIVOT_STATE desiredPivotState) {
        this.desiredPivotState = desiredPivotState;
        desiredPivotStateChanged = true;
    }

    public void setDesiredIntakeState(ALGAE_CATCHER_INTAKE_STATE desiredIntakeState) {
        this.desiredIntakeState = desiredIntakeState;
        desiredIntakeStateChanged = true;
    }

    public boolean isBeamBreakTriggered() {
//        if(RobotBase.isSimulation())
//            return false;

        return !algaeSensor.get();
    }

    /**
     * Reads actual outputs from intake motor
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
//        System.out.println(pivotMotor.getSensorPosition());
//        System.out.println(intakeMotor.getSensorVelocity());
//        System.out.println(desiredIntakeState.name());

        actualAlgaeCatcherVelocity = intakeMotor.getMotorOutputPercent();
        algaeCatcherCurrentDraw = intakeMotor.getMotorOutputCurrent();

        // get value once the value may change ever call, and you want to use
        // the same value for all the logic
        boolean beamBreak = isBeamBreakTriggered();
        if (beamBreak && desiredIntakeState == ALGAE_CATCHER_INTAKE_STATE.INTAKE) {
            desiredIntakeState = ALGAE_CATCHER_INTAKE_STATE.HOLD;
        }
        else if (!beamBreak && desiredIntakeState == ALGAE_CATCHER_INTAKE_STATE.INTAKE) {
            desiredIntakeState = ALGAE_CATCHER_INTAKE_STATE.INTAKE;
        }

//        System.out.println(isBeamBreakTriggered());

        if (desiredIntakeState == ALGAE_CATCHER_INTAKE_STATE.INTAKE){
            desiredPivotState = ALGAE_CATCHER_PIVOT_STATE.INTAKE;
        } else if (desiredIntakeState == ALGAE_CATCHER_INTAKE_STATE.HOLD){
            desiredPivotState = ALGAE_CATCHER_PIVOT_STATE.HOLD;
        } else if (desiredIntakeState == ALGAE_CATCHER_INTAKE_STATE.STOP){
            desiredPivotState = ALGAE_CATCHER_PIVOT_STATE.STOW;
        } else if (desiredIntakeState == ALGAE_CATCHER_INTAKE_STATE.OUTTAKE && desiredPivotState != ALGAE_CATCHER_PIVOT_STATE.REMOVE_ALGAE){
            desiredPivotState = ALGAE_CATCHER_PIVOT_STATE.OUTTAKE;
        }

        if (robotState.actualAlgaeCatcherIntakeState != desiredIntakeState) {
            robotState.actualAlgaeCatcherIntakeState = desiredIntakeState;
            desiredIntakeStateChanged = true;
        }

        if (robotState.actualAlgaeCatcherPivotState != desiredPivotState) {
            robotState.actualAlgaeCatcherPivotState = desiredPivotState;
            desiredPivotStateChanged = true;
        }

        if (intakeMotor.getMotorTemperature() >= 55) {
            SmartDashboard.putBoolean("Collector", false);
        }

        SmartDashboard.putBoolean("AlgaeCatcherBeamBreak", isBeamBreakTriggered());

        if (Constants.kLoggingRobot) {
            ((DoubleLogEntry) desStatesLogger).append(desiredAlgaeCatcherPower);
        }
    }

    /**
     * Writes outputs to intake motor
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (desiredIntakeStateChanged) {
            desiredIntakeStateChanged = false;
            switch (desiredIntakeState) {
                case STOP -> desiredAlgaeCatcherPower = 0;

                case INTAKE -> desiredAlgaeCatcherPower = algaeCollectSpeed;

                case HOLD -> desiredAlgaeCatcherPower = algaeHoldSpeed;

                case OUTTAKE -> desiredAlgaeCatcherPower = algaeReleaseSpeed;
            }
            // Good to log states to aid troubleshooting
            GreenLogger.log("Coral:" + desiredIntakeState + " Power:" + desiredAlgaeCatcherPower);
            intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, desiredAlgaeCatcherPower);
            algaeCatcherCurrentDrawLogger.append(desiredAlgaeCatcherPower);
        }

        robotState.algaeCatcherPivot.setAngle(robotState.algaeBaseAngle + pivotMotor.getSensorPosition() / algaeMotorRotationsPerDegree);

        if(desiredPivotStateChanged) {
            desiredPivotStateChanged = false;
            switch (desiredPivotState){
                case STOW -> {
                    desiredPosition = stowPosition;
                }
                case HOLD -> {
                    desiredPosition = holdPosition;
                }
                case INTAKE -> {
                    desiredPosition = intakePosition;
                }
                case OUTTAKE -> {
                    desiredPosition = outtakePosition;
                }
                case REMOVE_ALGAE -> {
                    desiredPosition = removeAlgaePosition;
                }
            }
            pivotMotor.set(GreenControlMode.MOTION_MAGIC_EXPO, MathUtil.clamp(desiredPosition, .25, 40));
        }
    }

    public boolean isAlgaeCatcherPivotInRange(){
        return Math.abs(pivotMotor.getSensorPosition() - desiredPosition) < 2;
    }

    public boolean isAlgaeCatcherIntakeInRange(){
        return Math.abs(intakeMotor.getMotorOutputPercent() - desiredAlgaeCatcherPower) < 5;
    }

    @Override
    public void zeroSensors() {
        pivotMotor.setSensorPosition(0);
    }

    @Override
    public void stop() {
        desiredIntakeState = ALGAE_CATCHER_INTAKE_STATE.STOP;
    }

    /**
     * Tests the collector subsystem, returns true if tests passed
     *
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        //TODO make this once the rest of the subsystem is done and tested
        return false;
    }

    /**
     * Returns the desired collector state
     *
     * @return desired collector state
     */
    public ALGAE_CATCHER_INTAKE_STATE getDesiredAlgaeCatcherIntakeState() {
        return desiredIntakeState;
    }

    public ALGAE_CATCHER_PIVOT_STATE getDesiredAlgaeCatcherPivotState() {
        return desiredPivotState;
    }

    /**
     * Returns the intake motor of the collector velocity
     *
     * @return intake velocity
     */
    public double getAlgaeCatcherVelocity() {
        return actualAlgaeCatcherVelocity;
    }

    /**
     * Base enum for collector
     */
    public enum ALGAE_CATCHER_INTAKE_STATE {
        STOP,
        INTAKE,
        HOLD,
        OUTTAKE
    }
    public enum ALGAE_CATCHER_PIVOT_STATE {
        STOW,
        HOLD,
        INTAKE,
        OUTTAKE,
        REMOVE_ALGAE
    }
}
