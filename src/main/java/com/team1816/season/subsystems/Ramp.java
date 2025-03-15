package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jakarta.inject.Inject;

public class Ramp extends Subsystem {
    /**Name*/
    private static final String NAME = "ramp";
    
    /**Componenets*/
    private final IGreenMotor rampMotor;
    
    /**States*/
    private Ramp.RAMP_STATE desiredRampState = Ramp.RAMP_STATE.STOW;

    private boolean rampOutputsChanged = false;
    private boolean offsetHasBeenApplied = false;

    private double rampCurrentDraw;

    private double desiredRampPosition = 0;
    private double actualRampPosition = 0;

    private double rampMotorRotationsPerUnit = factory.getConstant(NAME, "rampMotorRotationsPerUnit", 1);

    /**Constants*/

    private double rampStowPosition = factory.getConstant(NAME, "rampStowPosition", 1.0);
    private double rampL1FeederPosition = factory.getConstant(NAME, "rampL1FeederPosition", 1.0);
    private double rampOtherFeederPosition = factory.getConstant(NAME, "rampOtherFeederPosition", 1.0);
    private double rampScorePosition = factory.getConstant(NAME, "rampScorePosition", 1.0);
    private double rampHoldPosition = factory.getConstant(NAME, "rampHoldPosition", 1.0);
    private double rampEjectPosition = factory.getConstant(NAME, "rampHoldPosition", 1.0);

    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    @Inject
    public Ramp(Infrastructure inf, RobotState rs){
        super(NAME, inf, rs);
        rampMotor = factory.getMotor(NAME, "rampMotor");
        rampMotor.config_PeakOutputForward(0.1);
        rampMotor.config_PeakOutputReverse(-0.1);

        rampMotor.selectPIDSlot(0);



        if (RobotBase.isSimulation()) { //TODO:CHANGE THESE
            rampMotor.setMotionProfileMaxVelocity(12 / 0.05);
            rampMotor.setMotionProfileMaxAcceleration(12 / 0.08);
            ((GhostMotor) rampMotor).setMaxVelRotationsPerSec(240);
        }
    }

    /**
     * Sets the desired state of the collector
     *
     * @param desiredRampState RAMP_STATE
     */
    public void setDesiredState(RAMP_STATE desiredRampState){
        this.desiredRampState = desiredRampState;

        rampOutputsChanged = true;
    }
    
    /**
     * Reads actual outputs from intake motor
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        actualRampPosition = rampMotor.getSensorPosition();
        rampCurrentDraw = rampMotor.getMotorOutputCurrent();

        //TODO: Add Mechanism Ligaments if needed

        if (robotState.actualRampState != desiredRampState) {
            robotState.actualRampState = desiredRampState;
            rampOutputsChanged = true;
        }
    }
    /**
     * Writes outputs to shooter motors
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (rampOutputsChanged || offsetHasBeenApplied) {
            rampOutputsChanged = false;
            offsetHasBeenApplied = false;

            desiredRampPosition = getRampPosition(desiredRampState);
//            System.out.println("ramp state: "+desiredRampPosition+" Position: "+desiredRampPosition);
            rampMotor.set(GreenControlMode.POSITION_CONTROL, MathUtil.clamp(desiredRampPosition, -20, 0));
        }
    }
    
    public void offsetRamp(double offsetAmount){
        switch (desiredRampState) {
            case STOW -> rampStowPosition += offsetAmount;
            case L1_FEEDER -> rampL1FeederPosition += offsetAmount;
            case HOLD -> rampHoldPosition += offsetAmount;
            case OTHER_FEEDER -> rampOtherFeederPosition += offsetAmount;
            case SCORE -> rampScorePosition += offsetAmount;
            case EJECT_CORAL -> rampEjectPosition += offsetAmount;
        }
        offsetHasBeenApplied = true;
        GreenLogger.log("Ramp " + desiredRampState + " position set to " + getRampPosition(desiredRampState));
    }

    public boolean isRampInRange(){
        return Math.abs(rampMotor.getSensorPosition() - desiredRampPosition) < 2;
    }

    @Override
    public void zeroSensors() {rampMotor.setSensorPosition(0);}

    @Override
    public void stop() {

    }

//    public void setBraking(boolean braking) {
//        rampMotor.setNeutralMode(braking ? NeutralMode.Brake : NeutralMode.Coast);
//    }
    
    @Override
    public boolean testSubsystem() {
        //TODO make this once the rest of the subsystem is done and tested
        return false;
    }
    
    private double getRampPosition(Ramp.RAMP_STATE rampState) {
        return switch (rampState) {
            case STOW -> rampStowPosition;
            case L1_FEEDER -> rampL1FeederPosition;
            case HOLD -> rampHoldPosition;
            case OTHER_FEEDER -> rampOtherFeederPosition;
            case SCORE -> rampScorePosition;
            case EJECT_CORAL -> rampEjectPosition;
        };
    }

    /**
     * Returns the desired ramp state
     *
     * @return desired ramp state
     */
    public Ramp.RAMP_STATE getDesiredRampState() {return desiredRampState;}
    public Ramp.RAMP_STATE getDesisetredRampState() {return desiredRampState;}

    /**
     * Ramp enum
     */
    public enum RAMP_STATE{
        STOW,
        L1_FEEDER,
        OTHER_FEEDER,
        HOLD,
        SCORE,
        EJECT_CORAL
    }
}
