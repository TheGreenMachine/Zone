package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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
    private final IGreenMotor leadMotor;
    private final IGreenMotor followerMotor;
    private final IGreenMotor positionMotor;

    /**
     * Constants
     */
    private static final double neutralPosition = factory.getConstant(NAME,"neutralPosition",1.0);
    private static final double intakePosition = factory.getConstant(NAME,"intakePosition",1.0);
    private static final double holdPosition = factory.getConstant(NAME,"holdPosition",1.0);
    private static final double outtakePosition = factory.getConstant(NAME,"outtakePosition",1.0);


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
    private ALGAE_CATCHER_STATE desiredState = ALGAE_CATCHER_STATE.STOP;
    private POSITION_STATE desiredPositionState = POSITION_STATE.STOW;
    private double actualAlgaeCatcherPower = 0;
    private double desiredAlgaeCatcherPower = 0;
    private double algaeCatcherCurrentDraw = 0;
    private boolean outputsChanged = false;
    private boolean positionOutputsChanged = false;
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
        leadMotor = factory.getMotor(NAME, "algaeCatcherLeadMotor");
        followerMotor = factory.getMotor(NAME, "algaeCatcherFollowerMotor");
        positionMotor = factory.getMotor(NAME, "algaeCatcherPositionMotor");

        followerMotor.follow(leadMotor, true);

        algaeCollectSpeed = factory.getConstant(NAME, "algaeCollectSpeed", -0.5);
        algaeHoldSpeed = factory.getConstant(NAME, "algaeHoldSpeed", -0.1);
        algaeReleaseSpeed = factory.getConstant(NAME, "algaeReleaseSpeed", 0.25);

        SmartDashboard.putBoolean("AlgaeCollector", leadMotor.getMotorTemperature() < 55);

        positionMotor.selectPIDSlot(0);

        if (Constants.kLoggingRobot) {

            desStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "Collector/desiredAlgaeCatcherPower");
            GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "Collector/actualAlgaeCatcherPower"), leadMotor::getMotorOutputPercent);
            GreenLogger.addPeriodicLog(new DoubleLogEntry(DataLogManager.getLog(), "Collector/algaeCatcherCurrentDraw"), leadMotor::getMotorOutputCurrent);
        }
    }

    /**
     * Sets the desired state of the collector
     *
     * @param desiredState COLLECTOR_STATE
     */
    public void setDesiredState(ALGAE_CATCHER_STATE desiredState) {
        this.desiredState = desiredState;
        outputsChanged = true;
    }

    /**
     * Sets the desired state of the position
     *
     * @param desiredPositionState POSITION_STATE
     */
    public void setDesiredPositionState(POSITION_STATE desiredPositionState) {
        this.desiredPositionState = desiredPositionState;
        positionOutputsChanged = true;
    }
    /**
     * Reads actual outputs from intake motor
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        actualAlgaeCatcherPower = leadMotor.getMotorOutputPercent();
        algaeCatcherCurrentDraw = leadMotor.getMotorOutputCurrent();

        if (robotState.actualAlgaeCatcherState != desiredState) {
            robotState.actualAlgaeCatcherState = desiredState;
        }

        if (leadMotor.getMotorTemperature() >= 55) {
            SmartDashboard.putBoolean("Collector", false);
        }
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
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
                case STOP -> {
                    desiredAlgaeCatcherPower = 0;
                }
                case INTAKE -> {
                    desiredAlgaeCatcherPower = algaeCollectSpeed;
                }
                case HOLD -> {
                    desiredAlgaeCatcherPower = algaeHoldSpeed;
                }
                case OUTTAKE -> {
                    desiredAlgaeCatcherPower = algaeReleaseSpeed;
                }
            }
            leadMotor.set(GreenControlMode.PERCENT_OUTPUT, desiredAlgaeCatcherPower);
        }
        if(positionOutputsChanged) {
            positionOutputsChanged = false;
            switch (desiredPositionState){
                case STOW -> {
                    desiredPosition = neutralPosition;
                }
                case INTAKE -> {
                    desiredPosition = intakePosition;
                }
                case HOLD -> {
                    desiredPosition = holdPosition;
                }
                case OUTTAKE -> {
                    desiredPosition = outtakePosition;
                }
            }
        }
    }

    @Override
    public void zeroSensors() {
        //No implementation
    }

    @Override
    public void stop() {
        desiredState = ALGAE_CATCHER_STATE.STOP;
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
    public ALGAE_CATCHER_STATE getDesiredAlgaeCatcherState() {
        return desiredState;
    }

    /**
     * Returns the intake motor of the collector velocity
     *
     * @return intake velocity
     */
    public double getAlgaeCatcherVelocity() {
        return actualAlgaeCatcherPower;
    }

    /**
     * Base enum for collector
     */
    public enum ALGAE_CATCHER_STATE {
        STOP,
        INTAKE,
        HOLD,
        OUTTAKE
    }
    public enum POSITION_STATE {
        STOW,
        INTAKE,
        HOLD,
        OUTTAKE
    }
}
