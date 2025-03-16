package com.team1816.season.subsystems;

import com.google.inject.Singleton;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.GhostMotor;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import jakarta.inject.Inject;

@Singleton
public class Ramp extends Subsystem {
    /**Name*/
    private static final String NAME = "ramp";
    
    /**Componenets*/
    private final IGreenMotor rampMotor;
    
    /**States*/
    private RAMP_STATE desiredRampState = RAMP_STATE.L234_FEEDER;

    private boolean rampOutputsChanged = false;
    private boolean offsetHasBeenApplied = false;

    private double rampCurrentDraw;

    private double desiredRampPosition = 0;
    private double actualRampPosition = 0;

    private double rampMotorRotationsPerUnit = factory.getConstant(NAME, "rampMotorRotationsPerUnit", 1);

    /**Constants*/

    private double rampL1FeederPosition = factory.getConstant(NAME, "rampL1FeederPosition", 1.0);
    private double rampL234FeederPosition = factory.getConstant(NAME, "rampL234FeederPosition", 1.0);
    private double rampScorePosition = factory.getConstant(NAME, "rampScorePosition", 1.0);
    private double rampDislodgeCoralPosition = factory.getConstant(NAME, "rampDislodgeCoralPosition", 1.0);
    private double rampClimbPosition = factory.getConstant(NAME, "rampClimbPosition", 1.0);

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
            rampMotor.setMotionProfileMaxVelocity(30);
            rampMotor.setMotionProfileMaxAcceleration(8);
            ((GhostMotor) rampMotor).setMaxVelRotationsPerSec(40);
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
            rampMotor.set(GreenControlMode.POSITION_CONTROL, MathUtil.clamp(desiredRampPosition, -15, 5));
        }

        robotState.rampMechArm.setAngle(robotState.rampMechArmBaseAngle + rampMotor.getSensorPosition() / rampMotorRotationsPerUnit);
    }
    
    public void offsetRamp(double offsetAmount){
        switch (desiredRampState) {
            case L1_FEEDER -> rampL1FeederPosition += offsetAmount;
            case L234_FEEDER -> rampL234FeederPosition += offsetAmount;
            case SCORE -> rampScorePosition += offsetAmount;
            case DISLODGE_CORAL -> rampDislodgeCoralPosition += offsetAmount;
            case CLIMB -> rampClimbPosition += offsetAmount;
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
    
    private double getRampPosition(RAMP_STATE rampState) {
        return switch (rampState) {
            case L1_FEEDER -> rampL1FeederPosition;
            case L234_FEEDER -> rampL234FeederPosition;
            case SCORE -> rampScorePosition;
            case DISLODGE_CORAL -> rampDislodgeCoralPosition;
            case CLIMB -> rampClimbPosition;
        };
    }

    /**
     * Returns the desired ramp state
     *
     * @return desired ramp state
     */
    public RAMP_STATE getDesiredRampState() {return desiredRampState;}

    /**
     * Ramp enum
     */
    public enum RAMP_STATE{
        L1_FEEDER,
        L234_FEEDER,
        SCORE,
        DISLODGE_CORAL,
        CLIMB
    }
}
