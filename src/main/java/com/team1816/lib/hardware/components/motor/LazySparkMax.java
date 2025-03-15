package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team1816.lib.hardware.components.motor.configurations.FeedbackDeviceType;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.hardware.components.motor.configurations.MotionCurveType;
import com.team1816.lib.util.ConfigurationTranslator;
import com.team1816.lib.util.logUtil.GreenLogger;

import static com.revrobotics.spark.config.LimitSwitchConfig.Type.kNormallyOpen;

public class LazySparkMax extends SparkMax implements IGreenMotor {
    private final SparkMaxConfig sparkConfig;
    private RelativeEncoder encoder;

    protected String name = "";
    protected GreenControlMode currentControlMode = GreenControlMode.PERCENT_OUTPUT;
    protected GreenControlMode lastControlMode = null;
    protected double lastSet = Double.NaN;
    protected ClosedLoopSlot currentPIDSlot;

    protected SparkLimitSwitch forwardLimitSwitch, reverseLimitSwitch = null;

    protected double peakOutputForward, peakOutputBackward = -0;

    protected double voltageForCompensation = 0;
    protected boolean voltageCompensationEnabled = false;

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceNumber The device ID.
     * @param motorName The name of the motor
     */
    public LazySparkMax(int deviceNumber, String motorName) {
        super(deviceNumber, SparkLowLevel.MotorType.kBrushless);
        encoder = super.getEncoder();
        name = motorName;
        sparkConfig = new SparkMaxConfig();
        currentPIDSlot = ClosedLoopSlot.kSlot0;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public IGreenMotor.MotorType get_MotorType() {
        return IGreenMotor.MotorType.SparkMax;
    }

    @Override
    public void selectFeedbackSensor(FeedbackDeviceType deviceType) {
        GreenLogger.log("Cannot select feedback sensor on SparkMax");
//        encoder = configureRelativeEncoder(deviceType);
    }

    @Override
    public void selectFeedbackSensor(FeedbackDeviceType deviceType, int id) {
        selectFeedbackSensor(deviceType);
    }

    private RelativeEncoder configureRelativeEncoder(FeedbackDeviceType deviceType) { // hopefully this doesn't cause any problems
        return super.getEncoder();
//        return super.getEncoder(
//            ConfigurationTranslator.toSparkRelativeEncoderType(deviceType),
//            42
//        );
    }

    @Override
    public void configCurrentLimit(SupplyCurrentLimitConfiguration configuration) {
        configCurrentLimit((int) configuration.currentLimit);
    }

    @Override
    public void configCurrentLimit(int current) {
        sparkConfig.smartCurrentLimit(current);
        reconfigure();
    }

    @Override
    public void configStatorCurrentLimit(double current, boolean enable) {
        // not sure what this is, so i'm going to ignore it
    }

    @Override
    public void set(GreenControlMode controlMode, double demand) {
        currentControlMode = controlMode;
        if (demand != lastSet || currentControlMode != lastControlMode) {
            lastSet = demand;
            lastControlMode = currentControlMode;
            getClosedLoopController().setReference(
                    demand,
                    ConfigurationTranslator.toSparkMaxControlType(controlMode),
                    currentPIDSlot
            );
        }
    }

    @Override
    public void configForwardLimitSwitch(boolean normallyOpen) {
        sparkConfig.limitSwitch.forwardLimitSwitchType(normallyOpen ? kNormallyOpen : LimitSwitchConfig.Type.kNormallyClosed);
        reconfigure();
        forwardLimitSwitch = super.getForwardLimitSwitch();
    }

    @Override
    public void configReverseLimitSwitch(boolean normallyOpen) {
        sparkConfig.limitSwitch.reverseLimitSwitchType(normallyOpen ? kNormallyOpen : LimitSwitchConfig.Type.kNormallyClosed);
        reconfigure();
        reverseLimitSwitch = super.getReverseLimitSwitch();
    }

    @Override
    public boolean isLimitSwitchClosed(LimitSwitchDirection direction) {
        return direction == LimitSwitchDirection.FORWARD ? forwardLimitSwitch.isPressed() : reverseLimitSwitch.isPressed();
    }


    @Override
    public void neutralOutput() {
        super.stopMotor();
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        sparkConfig.idleMode(neutralMode == NeutralMode.Brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        reconfigure();
    }

    @Override
    public void setSensorPhase(boolean isInverted) {
        GreenLogger.log("Cannot invert sensor phase of a Spark in brushless mode");
        //If we ever have a spark controlling a brushed motor, the next line can be uncommented.
            //encoder.setInverted(isInverted); // This is NOT the same as a call to super.getInverted().
    }

    @Override
    public void setInvertedMotor(boolean isInvertedMotor) {
        sparkConfig.inverted(isInvertedMotor);
        reconfigure();
    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull) {
        sparkConfig.openLoopRampRate(secondsNeutralToFull);
        reconfigure();
    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull, int timeoutMs) {
        configOpenLoopRampRate(secondsNeutralToFull);
    }

    @Override
    public void configClosedLoopRampRate(double secondsNeutralToFull) {
        sparkConfig.closedLoopRampRate(secondsNeutralToFull);
        reconfigure();
    }

    @Override
    public void config_PeakOutputForward(double percentOut) {
        peakOutputForward = percentOut;
        sparkConfig.closedLoop.outputRange(peakOutputBackward, peakOutputForward, currentPIDSlot);
        reconfigure();
    }

    @Override
    public void config_PeakOutputForward(double percentOut, int timeoutMs) {
        config_PeakOutputForward(percentOut);
    }


    @Override
    public void config_PeakOutputReverse(double percentOut) {
        //Use negative values for backwards range
        peakOutputBackward = percentOut;
        sparkConfig.closedLoop.outputRange(peakOutputBackward, peakOutputForward, currentPIDSlot);
        reconfigure();
    }

    @Override
    public void config_PeakOutputReverse(double percentOut, int timeoutMs) {
        config_PeakOutputReverse(percentOut);
    }


    /**
     * @see <a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces">Documentation</a>
     */
    @Override
    public void config_NeutralDeadband(double deadbandPercent) {
        GreenLogger.log("Neutral deadband is only configurable through USB for Spark Max. Factory default is ±5%");

    }

    @Override
    public void enableClearPositionOnLimitF(boolean clearPosition, int timeoutMs) {
        //No functionality
    }

    @Override
    public void enableClearPositionOnLimitR(boolean clearPosition, int timeoutMs) {
        //No functionality
    }

    @Override
    public double getMotorOutputPercent() {
        return super.getAppliedOutput(); // We don't use get() because that is only supplied with set() and we skip over that for setReference()
    }

    @Override
    public double getMotorOutputVoltage() {
        return getMotorOutputPercent() * getBusVoltage(); //hate this but it's literally how BaseMotorController does it
    }

    @Override
    public double get_SupplyCurrent() {
        return super.getOutputCurrent();
    }

    @Override
    public double get_ClosedLoopOutput() {
        return 0;
    }

    @Override
    public boolean getInvertedMotor() {
        return configAccessor.getInverted();
    }

    @Override
    public double getSensorPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getSensorVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setSensorPosition(double sensorPosition) {
        encoder.setPosition(sensorPosition);
    }

    @Override
    public void setSensorPosition(double sensorPosition, int timeoutMs) {
        setSensorPosition(sensorPosition);
    }

    @Override
    public void enableLimitSwitches(boolean isEnabled) {
        //WHY DO LIMIT SWITCHES HAVE A TOGGLE PARAMETER BUT VOLTAGE COMPENSATION DOESNT
        if (isEnabled) {
            configForwardLimitSwitch(true);
            configReverseLimitSwitch(true);
        }
    }

    @Override
    public void configForwardSoftLimit(double forwardSoftLimit) {
        sparkConfig.softLimit.forwardSoftLimit(forwardSoftLimit);
        reconfigure();
    }

    @Override
    public void configReverseSoftLimit(double reverseSoftLimit) {
        sparkConfig.softLimit.reverseSoftLimit(reverseSoftLimit);
        reconfigure();
    }

    @Override
    public void enableForwardSoftLimit(boolean isEnabled) {
        sparkConfig.softLimit.forwardSoftLimitEnabled(isEnabled);
        reconfigure();
    }

    @Override
    public void enableReverseSoftLimit(boolean isEnabled) {
        sparkConfig.softLimit.reverseSoftLimitEnabled(isEnabled);
        reconfigure();
    }

    @Override
    public void enableSoftLimits(boolean isEnabled) {
        enableForwardSoftLimit(isEnabled);
        enableReverseSoftLimit(isEnabled);
    }

    @Override
    public void set_kP(int pidSlotID, double kP) {
        sparkConfig.closedLoop.p(kP, getPIDSlot(pidSlotID));
        reconfigure();
    }

    @Override
    public void set_kI(int pidSlotID, double kI) {
        sparkConfig.closedLoop.i(kI, getPIDSlot(pidSlotID));
        reconfigure();
    }

    @Override
    public void set_kD(int pidSlotID, double kD) {
        sparkConfig.closedLoop.d(kD, getPIDSlot(pidSlotID));
        reconfigure();
    }

    @Override
    public void set_kV(int pidSlotID, double kV) {
        sparkConfig.closedLoop.velocityFF(kV, getPIDSlot(pidSlotID));
        reconfigure();
    }

    @Override
    public void set_kS(int pidSlotID, double kS) {
        // Does not do anything
    }

    @Override
    public void set_kA(int pidSlotID, double kA) {
        // Does not do anything
    }

    @Override
    public void set_kG(int pidSlotID, double kG) {
        // Does not do anything
    }

    @Override
    public void selectPIDSlot(int pidSlotID) {
        currentPIDSlot = getPIDSlot(pidSlotID);
    }

    @Override
    public void setPeakOutputClosedLoop(int pidSlotID, double peakOutput) {
        sparkConfig.closedLoop.outputRange(-peakOutput, peakOutput, getPIDSlot(pidSlotID));
        reconfigure();
    }

    @Override
    public double get_ClosedLoopError() {
        // This isn't worth implementing as of 2024-25 because we aren't using rev motors for driving or anything that needs that much precision.
        // If anyone in the future wants to take a stab at it go ahead:
            //This is theoretically possible in a few ways
                // The actual firmware implementation is here https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control but error is not retrievable
                    //if we figured out what pv meant then we could calc it ourselves
                // Could also reverse engineer the output from the PID equation but that could potentially be really slow
        return Double.NaN;
    }

    @Override
    public void setMotionProfileMaxVelocity(double maxVelocity) {
        closedLoopController.setReference(maxVelocity, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setMotionProfileMaxAcceleration(double maxAcceleration) {
        sparkConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
        reconfigure();
    }

    @Override
    public void configMotionCurve(MotionCurveType motionCurveType, int curveStrength) { // TODO: re-implement this when s-curves are supported
        sparkConfig.closedLoop.maxMotion.positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal);
        reconfigure();
    }

    @Override
    public boolean hasResetOccurred() {
        GreenLogger.log("SparkMax does not track resets."); //Apparently tracks resets as a fault but I'm not implementing a method for that
        return false;
    }

    @Override
    public int getDeviceID() {
        return super.getDeviceId();
    }

    @Override
    public double getMotorOutputCurrent() {
        return super.getOutputCurrent();
    }

    @Override
    public GreenControlMode get_ControlMode() {
        return currentControlMode;
    }

    @Override
    public void follow(IGreenMotor leader, boolean opposeLeaderDirection) {
        if (leader instanceof LazySparkMax) {
            sparkConfig.follow((SparkMax) leader, opposeLeaderDirection);
        } else {
//            sparkConfig.follow(leader.getDeviceID(), opposeLeaderDirection);
            GreenLogger.log("Sparks cannot follow non-Sparks");
        }

        reconfigure();
    }

    @Override
    public void restore_FactoryDefaults(int timeoutMs) {
        //This method doesn't do anything because sparkmax restorefactorydefaults also resets USB-exclusive settings and that's annoying to deal with
    }

    @Override
    public void configControlFramePeriod(ControlFrame controlFrame, int periodms) {
        super.setControlFramePeriodMs(periodms);
    }

    private void reconfigure() {
        this.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private ClosedLoopSlot getPIDSlot(int pidSlotID) {
        return switch (pidSlotID) {
            case 0 -> ClosedLoopSlot.kSlot0;
            case 1 -> ClosedLoopSlot.kSlot1;
            case 2 -> ClosedLoopSlot.kSlot2;
            case 3 -> ClosedLoopSlot.kSlot3;
            default -> throw new IllegalStateException("Unexpected value: " + pidSlotID + "; PID slot IDS may only be 0-3");
        };
    }
}
