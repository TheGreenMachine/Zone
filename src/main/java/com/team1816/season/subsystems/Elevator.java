package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.GhostMotor;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;

@Singleton
public class Elevator extends Subsystem {

    /**
     * Name
     */
    private static final String NAME = "elevator";


    /**
     * Components
     */
    private final IGreenMotor elevatorMotor;

    /**
     * States
     */

    private ELEVATOR_STATE desiredElevatorState = ELEVATOR_STATE.FEEDER;

    private boolean elevatorOutputsChanged = false;

    private double elevatorCurrentDraw;

    private double desiredElevatorPosition = 0;
    private double actualElevatorPosition = 0;

    private double elevatorMotorRotationsPerUnit = factory.getConstant(NAME, "elevatorMotorRotationsPerUnit", 1);

    /**
     * Constants
     */

    private final double elevatorFeederPosition = factory.getConstant(NAME, "elevatorFeederPosition", 1.0);
    private final double elevatorL1Position = factory.getConstant(NAME, "elevatorL1Position", 1.0);
    private final double elevatorL2Position = factory.getConstant(NAME, "elevatorL2Position", 1.0);
    private final double elevatorL3Position = factory.getConstant(NAME, "elevatorL3Position", 1.0);
    private final double elevatorL4Position = factory.getConstant(NAME, "elevatorL4Position", 1.0);

//    private final boolean opposeLeaderDirection = ((int) factory.getConstant(NAME, "invertFollowerMotor", 0)) == 1;

    /**
     * Base constructor needed to instantiate a shooter
     *
     * @param inf Infrastructure
     * @param rs  RobotState
     */
    @Inject
    public Elevator(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        elevatorMotor = factory.getMotor(NAME, "elevatorMotor");

        elevatorMotor.selectPIDSlot(0);

        if (RobotBase.isSimulation()) {
            elevatorMotor.setMotionProfileMaxVelocity(12 / 0.05);
            elevatorMotor.setMotionProfileMaxAcceleration(12 / 0.08);
            ((GhostMotor) elevatorMotor).setMaxVelRotationsPerSec(240);
        }
    }

    /**
     * Sets the desired state of the elevator
     *
     * @param desiredElevatorState ELEVATOR_STATE
     */
    public void setDesiredState(ELEVATOR_STATE desiredElevatorState) {
        this.desiredElevatorState = desiredElevatorState;

        elevatorOutputsChanged = true;
    }

    /**
     * Reads actual outputs from shooter motors
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        actualElevatorPosition = elevatorMotor.getSensorPosition();

        elevatorCurrentDraw = elevatorMotor.getMotorOutputCurrent();

        robotState.elevatorMechArm.setLength(elevatorMotor.getSensorPosition() / elevatorMotorRotationsPerUnit);

//        System.out.print(robotState.isCoralBeamBreakTriggered);
        if(!robotState.isCoralBeamBreakTriggered && robotState.actualCoralArmIntakeState != CoralArm.INTAKE_STATE.OUTTAKE) {
            desiredElevatorState = ELEVATOR_STATE.FEEDER;
        }

        if (robotState.actualElevatorState != desiredElevatorState) {
            robotState.actualElevatorState = desiredElevatorState;
        }
    }

    /**
     * Writes outputs to shooter motors
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (elevatorOutputsChanged) {
            elevatorOutputsChanged = false;
            switch (desiredElevatorState) {
                case FEEDER -> {
                    desiredElevatorPosition = elevatorFeederPosition;
                }
                case L1 -> {
                    desiredElevatorPosition = elevatorL1Position;
                }
                case L2 -> {
                    desiredElevatorPosition = elevatorL2Position;
                }
                case L3 -> {
                    desiredElevatorPosition = elevatorL3Position;
                }
                case L4 -> {
                    desiredElevatorPosition = elevatorL4Position;
                }
            }
            elevatorMotor.set(GreenControlMode.MOTION_MAGIC_EXPO, MathUtil.clamp(desiredElevatorPosition, 0, 67));
        }
    }

    public boolean isElevatorInRange(){
        return Math.abs(elevatorMotor.getSensorPosition() - desiredElevatorPosition) < 1;
    }

    @Override
    public void zeroSensors() {
        elevatorMotor.setSensorPosition(0);
    }

    @Override
    public void stop() {

    }

    public void setBraking(boolean braking) {
        elevatorMotor.setNeutralMode(braking ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public boolean testSubsystem() {
        //TODO eventually.
        return false;
    }

    /**
     * Returns the desired elevator state
     *
     * @return desired elevator state
     */
    public ELEVATOR_STATE getDesiredElevatorState() {return desiredElevatorState;}
    public ELEVATOR_STATE getDesisetredElevatorState() {return desiredElevatorState;}

    /**
     * Elevator enum
     */
    public enum ELEVATOR_STATE {
        FEEDER,
        L1,
        L2,
        L3,
        L4
    }
}
