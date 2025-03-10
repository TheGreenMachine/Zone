package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1816.core.Robot;
import com.team1816.lib.hardware.components.motor.configurations.FeedbackDeviceType;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.hardware.components.motor.configurations.MotionCurveType;
import com.team1816.lib.util.driveUtil.DriveConversions;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import static java.lang.Double.NaN;

public class GhostMotor implements IGreenMotor {
    protected String name = "";

    /**
     * Characterization
     */
    private double maxVelRotationsPerSec;
    private final int absInitOffset;
    private int fwdLimit;
    private int revLimit;
    private boolean usingLimit = false;
    private final int absMotorPPR = 4096;
    /**
     * State
     */
    private GreenControlMode controlMode;
    private final double[] desiredDemand = new double[]{0, 0, 0}; // 0: %out, 1: vel, 2: pos, 3: Motion magic
    private final double[] actualOutput = new double[]{0, 0, 0}; // 0: %out, 1: vel, 2: pos, 3: Motion Magic
    protected double lastPos = 0;

    private double motionMagicCruiseVel;
    private double motionMagicAccel;

    protected double lastUpdate = 0;

    public GhostMotor(double maxVelMPS, int absInitOffset, String motorName) {
        this.absInitOffset = absInitOffset;
        this.maxVelRotationsPerSec = DriveConversions.metersToRotations(maxVelMPS);
        name = motorName;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public MotorType get_MotorType() {
        return MotorType.GHOST;
    }

    @Override
    public void selectFeedbackSensor(FeedbackDeviceType deviceType) {

    }

    @Override
    public void selectFeedbackSensor(FeedbackDeviceType deviceType, int id) {

    }

    @Override
    public void configCurrentLimit(SupplyCurrentLimitConfiguration configuration) {

    }


    @Override
    public void configCurrentLimit(int current) {

    }

    @Override
    public void configStatorCurrentLimit(double current, boolean enable) {

    }

    @Override
    public double getMotorOutputCurrent() {
        return 0;
    }

    @Override
    public void set(GreenControlMode Mode, double demand) {
        processSet(Mode, demand);
    }

    private void processSet(GreenControlMode controlModeDemand, double demand) {
        // setting desired demand
        if (controlModeDemand == GreenControlMode.PERCENT_OUTPUT) {
            desiredDemand[0] = demand;
            desiredDemand[1] = NaN;
            desiredDemand[2] = NaN;
        } else if (controlModeDemand == GreenControlMode.VELOCITY_CONTROL) {
            this.desiredDemand[0] = NaN;
            this.desiredDemand[1] = demand;
            this.desiredDemand[2] = NaN;
        } else if (controlModeDemand == GreenControlMode.POSITION_CONTROL || controlModeDemand == GreenControlMode.MOTION_MAGIC || controlModeDemand == GreenControlMode.MOTION_MAGIC_EXPO) {
            this.desiredDemand[0] = NaN;
            this.desiredDemand[1] = NaN;
            this.desiredDemand[2] = demand;
        } else {
            GreenLogger.log("no support for Mode " + controlModeDemand + " in GhostMotor!");
            return;
        }
        controlMode = controlModeDemand;
    }

    private void updateActValues() {
        // don't make unnecessary calculations if robot not in sim
        if (RobotBase.isReal()) {
            return;
        }

        // whether motor needs to calculate new numbers - this
        double timeNow = Timer.getFPGATimestamp();
        double dtBetweenCallsSec = (timeNow - lastUpdate);
        double dtBetweenCallsMs = dtBetweenCallsSec * 1000;


        if (dtBetweenCallsSec < (Robot.robotDt / 1000) * 0.75) {
            lastUpdate = timeNow;
            return;
        }

        lastUpdate = timeNow;

        // setting actual output
        if (controlMode == GreenControlMode.PERCENT_OUTPUT) {
            actualOutput[0] = desiredDemand[0];
            actualOutput[1] = desiredDemand[0] * maxVelRotationsPerSec;
            actualOutput[2] = lastPos + (actualOutput[1] * dtBetweenCallsSec);
        } else if (controlMode == GreenControlMode.VELOCITY_CONTROL) {
            actualOutput[0] = desiredDemand[1] / maxVelRotationsPerSec;
            actualOutput[1] = desiredDemand[1];
            actualOutput[2] = lastPos + (actualOutput[1] * dtBetweenCallsSec);
        } else if (controlMode == GreenControlMode.POSITION_CONTROL) {
            double desaturatedVel = Math.signum(desiredDemand[2] - lastPos) * Math.min(maxVelRotationsPerSec, Math.abs(desiredDemand[2] - lastPos) / dtBetweenCallsSec);

            actualOutput[0] = desaturatedVel / maxVelRotationsPerSec;
            actualOutput[1] = desaturatedVel;
            actualOutput[2] = lastPos + (actualOutput[1] * dtBetweenCallsSec);
        } else if (controlMode == GreenControlMode.MOTION_MAGIC || controlMode == GreenControlMode.MOTION_MAGIC_EXPO) {
            // not accounting for accel rn - just using motionMagicCruiseVel
            double accelAccountedVel = Math.min(motionMagicCruiseVel, Math.abs(actualOutput[1]) + (motionMagicAccel * dtBetweenCallsSec));
            double desaturatedVel = Math.signum(desiredDemand[2] - lastPos) * Math.min(accelAccountedVel, Math.abs(desiredDemand[2] - lastPos) / dtBetweenCallsSec);
            actualOutput[0] = desaturatedVel / maxVelRotationsPerSec;
            actualOutput[1] = desaturatedVel;
            actualOutput[2] = lastPos + (actualOutput[1] / dtBetweenCallsMs);
        }

        if (usingLimit) {
            if (actualOutput[2] >= fwdLimit) {
                actualOutput[0] = 0;
                actualOutput[1] = 0;
                actualOutput[2] = fwdLimit;
            } else if (actualOutput[2] <= revLimit) {
                actualOutput[0] = 0;
                actualOutput[1] = 0;
                actualOutput[2] = revLimit;
            }
        }

        lastPos = actualOutput[2];
    }

    @Override
    public void configForwardLimitSwitch(boolean normallyOpen) {

    }

    @Override
    public void configReverseLimitSwitch(boolean normallyOpen) {

    }

    @Override
    public boolean isLimitSwitchClosed(LimitSwitchDirection direction) {
        return true;
    }

    @Override
    public void neutralOutput() {

    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {

    }

    @Override
    public void setSensorPhase(boolean isInverted) {

    }

    @Override
    public void setInvertedMotor(boolean isInverted) {

    }

    @Override
    public boolean getInvertedMotor() {
        return false;
    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull) {

    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull, int timeoutMs) {

    }


    @Override
    public void configClosedLoopRampRate(double secondsNeutralToFull) {

    }

    @Override
    public void config_PeakOutputForward(double percentOut) {

    }

    @Override
    public void config_PeakOutputForward(double percentOut, int timeoutMs) {

    }


    @Override
    public void config_PeakOutputReverse(double percentOut) {

    }

    @Override
    public void config_PeakOutputReverse(double percentOut, int timeoutMs) {

    }

    @Override
    public void config_NeutralDeadband(double deadbandPercent) {

    }

    @Override
    public void enableClearPositionOnLimitF(boolean clearPosition, int timeoutMs) {

    }

    @Override
    public void enableClearPositionOnLimitR(boolean clearPosition, int timeoutMs) {

    }

    @Override
    public double getBusVoltage() {
        return 12;
    }

    @Override
    public double getMotorOutputPercent() {
        updateActValues();
        return actualOutput[0];
    }

    @Override
    public double getMotorOutputVoltage() {
        return getMotorOutputPercent() * RobotController.getBatteryVoltage();
    }

    @Override
    public double get_SupplyCurrent() {
        return 0;
    }

    @Override
    public double get_ClosedLoopOutput() {
        return 0;
    }

    @Override
    public double getMotorTemperature() {
        return 0;
    }

    @Override
    public double getSensorPosition() {
        updateActValues();
        return actualOutput[2];
    }

    @Override
    public double getSensorVelocity() {
        updateActValues();
        return actualOutput[1];
    }

    @Override
    public void setSensorPosition(double sensorPosition) {
        processSet(GreenControlMode.POSITION_CONTROL, sensorPosition);
    }

    @Override
    public void setSensorPosition(double sensorPosition, int timeoutMs) {
        setSensorPosition(sensorPosition);
    }


    @Override
    public void enableLimitSwitches(boolean isEnabled) {

    }

    @Override
    public void configForwardSoftLimit(double forwardSoftLimit) {
        usingLimit = true;
        fwdLimit = (int) forwardSoftLimit;
    }

    @Override
    public void configReverseSoftLimit(double reverseSoftLimit) {
        usingLimit = true;
        revLimit = (int) reverseSoftLimit;
    }

    @Override
    public void enableForwardSoftLimit(boolean isEnabled) {
        usingLimit = isEnabled;
    }

    @Override
    public void enableReverseSoftLimit(boolean isEnabled) {
        usingLimit = isEnabled;
    }

    @Override
    public void enableSoftLimits(boolean isEnabled) {
        usingLimit = isEnabled;
    }

    @Override
    public void set_kP(int pidSlotID, double kP) {

    }

    @Override
    public void set_kI(int pidSlotID, double kI) {

    }

    @Override
    public void set_kD(int pidSlotID, double kD) {

    }

    @Override
    public void set_kV(int pidSlotID, double kV) {

    }

    @Override
    public void set_kS(int pidSlotID, double kS) {

    }

    @Override
    public void set_kA(int pidSlotID, double kS) {

    }

    @Override
    public void set_kG(int pidSlotID, double kS) {

    }

    @Override
    public void selectPIDSlot(int pidSlotID) {

    }

    @Override
    public void setPeakOutputClosedLoop(int pidSlotID, double peakOutput) {

    }

    @Override
    public double get_ClosedLoopError() {
        return 0;
    }

    @Override
    public void setMotionProfileMaxVelocity(double maxVelocity) {
        motionMagicCruiseVel = maxVelocity;
    }

    public void setMaxVelRotationsPerSec(double maxVelRotationsPerSec) {
        this.maxVelRotationsPerSec = maxVelRotationsPerSec;
    }

    @Override
    public void setMotionProfileMaxAcceleration(double maxAcceleration) {
        motionMagicAccel = maxAcceleration;
    }

    @Override
    public void configMotionCurve(MotionCurveType motionCurveType, int curveStrength) {

    }

    @Override
    public boolean hasResetOccurred() {
        return false;
    }

    @Override
    public int getDeviceID() {
        return -1;
    }

    @Override
    public GreenControlMode get_ControlMode() {
        return controlMode;
    }

    @Override
    public void follow(IGreenMotor leader, boolean opposeLeaderDirection) {

    }

    @Override
    public void restore_FactoryDefaults(int timeoutMs) {

    }

    @Override
    public boolean isFollower() {
        return false;
    }


    @Override
    public void configControlFramePeriod(ControlFrame controlFrame, int periodms) {

    }
}
