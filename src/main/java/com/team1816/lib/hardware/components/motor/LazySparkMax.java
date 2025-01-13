package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.*;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.team1816.lib.hardware.components.motor.configurations.*;
import com.team1816.lib.util.logUtil.GreenLogger;


/**
 * @deprecated Sparks are not supported as of 2025. Check
 * <a href="https://github.com/TheGreenMachine/Zen/blob/main/src/main/java/com/team1816/lib/hardware/components/motor/LazySparkMax.java">Zen's LazySparkMax code</a>
 * for original code.
 */
@Deprecated(since = "Sparks are no longer supported as of 2025")
public class LazySparkMax extends SparkMax implements IGreenMotor {
    public LazySparkMax(int deviceNumber, String motorName) {
        super(deviceNumber, SparkLowLevel.MotorType.kBrushed);
        throw new UnsupportedOperationException();
    }

    @Override
    public String getName() {
        throw new UnsupportedOperationException();
    }

    @Override
    public IGreenMotor.MotorType get_MotorType() {
        return IGreenMotor.MotorType.SparkMax;
    }

    @Override
    public void selectFeedbackSensor(FeedbackDeviceType deviceType) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void selectFeedbackSensor(FeedbackDeviceType deviceType, int id) {
        throw new UnsupportedOperationException();
    }

    private RelativeEncoder configureRelativeEncoder(FeedbackDeviceType deviceType) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configCurrentLimit(SupplyCurrentLimitConfiguration configuration) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configCurrentLimit(int current) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configStatorCurrentLimit(double current, boolean enable) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void set(GreenControlMode controlMode, double demand) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configForwardLimitSwitch(boolean normallyOpen) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configReverseLimitSwitch(boolean normallyOpen) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean isLimitSwitchClosed(LimitSwitchDirection direction) {
        throw new UnsupportedOperationException();
    }


    @Override
    public void neutralOutput() {
        super.stopMotor();
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setSensorPhase(boolean isInverted) {
        GreenLogger.log("Cannot invert sensor phase of a Spark in brushless mode");
        //If we ever have a spark controlling a brushed motor, the next line can be uncommented.
        //encoder.setInverted(isInverted); // This is NOT the same as a call to super.getInverted().
    }

    @Override
    public void setInvertedMotor(boolean isInvertedMotor) {

    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull, int timeoutMs) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configClosedLoopRampRate(double secondsNeutralToFull) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void config_PeakOutputForward(double percentOut) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void config_PeakOutputForward(double percentOut, int timeoutMs) {
        throw new UnsupportedOperationException();
    }


    @Override
    public void config_PeakOutputReverse(double percentOut) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void config_PeakOutputReverse(double percentOut, int timeoutMs) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void config_NeutralDeadband(double deadbandPercent) {
        throw new UnsupportedOperationException();

    }

    @Override
    public void enableClearPositionOnLimitF(boolean clearPosition, int timeoutMs) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void enableClearPositionOnLimitR(boolean clearPosition, int timeoutMs) {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getMotorOutputPercent() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getMotorOutputVoltage() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double get_SupplyCurrent() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double get_ClosedLoopOutput() {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean getInvertedMotor() {
        return false;
    }

    @Override
    public double getSensorPosition() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getSensorVelocity() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setSensorPosition(double sensorPosition) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setSensorPosition(double sensorPosition, int timeoutMs) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void enableLimitSwitches(boolean isEnabled) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configForwardSoftLimit(double forwardSoftLimit) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configReverseSoftLimit(double reverseSoftLimit) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void enableForwardSoftLimit(boolean isEnabled) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void enableReverseSoftLimit(boolean isEnabled) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void enableSoftLimits(boolean isEnabled) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void set_kP(int pidSlotID, double kP) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void set_kI(int pidSlotID, double kI) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void set_kD(int pidSlotID, double kD) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void set_kF(int pidSlotID, double kF) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void selectPIDSlot(int pidSlotID) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void set_iZone(int pidSlotID, double iZone) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configAllowableErrorClosedLoop(int pidSlotID, double allowableError) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPeakOutputClosedLoop(int pidSlotID, double peakOutput) {
        throw new UnsupportedOperationException();
    }

    @Override
    public double get_ClosedLoopError() {
        // This isn't worth implementing as of 2023-24 because we aren't using rev motors for driving or anything that needs that much precision.
        // If anyone in the future wants to take a stab at it go ahead:
        //This is theoretically possible in a few ways
        // The actual firmware implementation is here https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control but error is not retrievable
        //if we figured out what pv meant then we could calc it ourselves
        // Could also reverse engineer the output from the PID equation but that could potentially be really slow
        return Double.NaN;
    }

    @Override
    public void setMotionProfileMaxVelocity(double maxVelocity) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMotionProfileMaxAcceleration(double maxAcceleration) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void configMotionCurve(MotionCurveType motionCurveType, int curveStrength) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean hasResetOccurred() {
        GreenLogger.log("SparkMax does not track resets."); //Apparently tracks resets as a fault but I'm not implementing a method for that
        return false;
    }

    @Override
    public int getDeviceID() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double getMotorOutputCurrent() {
        throw new UnsupportedOperationException();
    }

    @Override
    public GreenControlMode get_ControlMode() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void follow(IGreenMotor leader, boolean opposeLeaderDirection) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void restore_FactoryDefaults(int timeoutMs) {
        //This method doesn't do anything because sparkmax restorefactorydefaults also resets USB-exclusive settings and that's annoying to deal with
    }

    @Override
    public void configControlFramePeriod(ControlFrame controlFrame, int periodms) {
        throw new UnsupportedOperationException();
    }

}
