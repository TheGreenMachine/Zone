package com.team1816.season.subsystems;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeArm extends Subsystem {

    /**
     * Name
     */
    private static final String NAME = "algaeArm";

    /**
     * Components
     */
    private final IGreenMotor algaeArmMotor;

    /**
     * Properties
     */
    public final double algaeArmMotorSpeed;

    /**
     * States
     */
    private ALGAE_ARM_STATE desiredState = ALGAE_ARM_STATE.OFF;
    private double actualAlgaeArmMotorPower = 0;
    private double desiredAlgaeArmMotorPower = 0;
    private boolean outputsChanged = false;

    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param name String
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    public AlgaeArm(String name, Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        algaeArmMotor = factory.getMotor(NAME, "algaeArmMotor");
        algaeArmMotorSpeed = factory.getConstant(NAME, "algaeArmMotorSpeed", 0.5);
    }

    /**
     * Sets the desired state of the algae arm motor
     *
     * @param desiredState ALGAE_ARM_STATE
     */
    public void setDesiredState(ALGAE_ARM_STATE desiredState) {
        this.desiredState = desiredState;
        outputsChanged = true;
    }

    /**
     * Reads actual outputs from algae arm motor
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        actualAlgaeArmMotorPower = algaeArmMotor.getMotorOutputPercent();
        if (robotState.actualAlgaeArmState != desiredState) {
            robotState.actualAlgaeArmState = desiredState;
        }
    }

    /**
     * Writes outputs to algae arm motor
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredState) {
                case ON -> {
                    desiredAlgaeArmMotorPower = algaeArmMotorSpeed;
                }
                case OFF -> {
                    desiredAlgaeArmMotorPower = 0;
                }
            }
            algaeArmMotor.set(GreenControlMode.PERCENT_OUTPUT, desiredAlgaeArmMotorPower);
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {
        desiredState = ALGAE_ARM_STATE.OFF;
    }

    @Override
    public boolean testSubsystem() {
        return false;
    }

    /**
     * Returns the desired algae arm state
     *
     * @return desired algae arm state
     */
    public ALGAE_ARM_STATE getDesiredAlgaeArmState() {
        return desiredState;
    }

    /**
     * Returns the algae arm motor velocity
     *
     * @return algae arm motor velocity
     */
    public double getAlgaeArmMotorVelocity() {
        return actualAlgaeArmMotorPower;
    }

    /**
     * Base enum for collector
     */
    public enum ALGAE_ARM_STATE {
        ON,
        OFF
    }
}
