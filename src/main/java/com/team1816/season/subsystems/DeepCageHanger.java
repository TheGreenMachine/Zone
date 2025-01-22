package com.team1816.season.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.GhostMotor;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.LazyTalonFX;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jakarta.inject.Inject;
import jakarta.inject.Singleton;

import java.util.Optional;

@Singleton
public class DeepCageHanger extends Subsystem {

    /**
     * Name
     */
    private static final String NAME = "elevator";


    /**
     * Components
     */
    private final IGreenMotor deepCageHangerMotor;
    private final IGreenMotor deepCageHangerFollowMotor;

    /**
     * States
     */

    private DEEPCAGEHANGER_STATE desiredDeepCageHangerState = DEEPCAGEHANGER_STATE.ETHAN;

    private boolean deepCageHangerOutputsChanged = false;

    private double deepCageHangerCurrentDraw;

    private double desiredDeepCageHangerPosition = 0;
    private double actualDeepCageHangerPosition = 0;
    private double actualDeepCageHangerDegrees = 0;



    /**
     * Constants
     */

    private final double deepCageHangerEthanPosition = factory.getConstant(NAME, "deepCageHangerEthanPosition", 1.0);
    private final double deepCageHangerLiaoPosition = factory.getConstant(NAME, "deepCageHangerLiaoPosition", 1.0);

    private final boolean opposeLeaderDirection = ((int) factory.getConstant(NAME, "invertFollowerMotor", 0)) == 1;

    /**
     * Base constructor needed to instantiate a shooter
     *
     * @param inf Infrastructure
     * @param rs  RobotState
     */
    @Inject
    public DeepCageHanger(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        deepCageHangerMotor = factory.getMotor(NAME, "elevatorMotor");
        deepCageHangerFollowMotor = factory.getFollowerMotor(NAME, "elevatorFollowMotor", deepCageHangerMotor, opposeLeaderDirection);

        deepCageHangerMotor.selectPIDSlot(2);

        if (RobotBase.isSimulation()) {
            deepCageHangerMotor.setMotionProfileMaxVelocity(12 / 0.05);
            deepCageHangerMotor.setMotionProfileMaxAcceleration(12 / 0.08);
            ((GhostMotor) deepCageHangerMotor).setMaxVelRotationsPerSec(240);
        }
    }

    public void setDesiredDeepCageHangerState(DEEPCAGEHANGER_STATE desiredDeepCageHangerState) {
        this.desiredDeepCageHangerState = desiredDeepCageHangerState;
        deepCageHangerOutputsChanged = true;
    }

    public void setDesiredState(DEEPCAGEHANGER_STATE desiredElevatorState) {
        this.desiredDeepCageHangerState = desiredElevatorState;

        deepCageHangerOutputsChanged = true;
    }

    /**
     * Reads actual outputs from shooter motors
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        actualDeepCageHangerPosition = deepCageHangerMotor.getSensorPosition();

        deepCageHangerCurrentDraw = deepCageHangerMotor.getMotorOutputCurrent();

        if (robotState.actualDeepCageHangerState != desiredDeepCageHangerState) {
            robotState.actualDeepCageHangerState = desiredDeepCageHangerState;
        }
    }

    /**
     * Writes outputs to shooter motors
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (deepCageHangerOutputsChanged) {
            deepCageHangerOutputsChanged = false;
            switch (desiredDeepCageHangerState) {
                case ETHAN -> {
                    desiredDeepCageHangerPosition = deepCageHangerEthanPosition;
                }
                case LIAO -> {
                    desiredDeepCageHangerPosition = deepCageHangerLiaoPosition;
                }
            }
            deepCageHangerMotor.set(GreenControlMode.MOTION_MAGIC_EXPO, MathUtil.clamp(desiredDeepCageHangerPosition, 1.5, 35));
        }
    }

    public double getActualDeepCageHangerPosition () {
        return deepCageHangerMotor.getSensorPosition();
    }

    @Override
    public void zeroSensors() {
        deepCageHangerMotor.setSensorPosition(0, 50);
    }

    @Override
    public void stop() {

    }

    public void setBraking(boolean braking) {
        deepCageHangerMotor.setNeutralMode(braking ? NeutralMode.Brake : NeutralMode.Coast);
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
    public DEEPCAGEHANGER_STATE getDesiredElevatorState() {
        return desiredDeepCageHangerState;
    }

    /**
     * Elevator enum
     */
    public enum DEEPCAGEHANGER_STATE {
        ETHAN,
        LIAO
    }
}
