package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

@Singleton
public class Pneumatic extends Subsystem {
    /**
     * Properties
     */
    private static final String NAME = "pneumatic";

    /**
     * Components
     */
    private final DoubleSolenoid doublePneumatic;

    /**
     * State
     */
    private Pneumatic.PNEUMATIC_STATE desiredPneumaticState = PNEUMATIC_STATE.OFF;
    private Pneumatic.PNEUMATIC_STATE actualPneumaticState = PNEUMATIC_STATE.OFF;

    private boolean pneumaticOutputsChanged = true;

    /**
     * Constants
     */
    private final double pneumaticOn = factory.getConstant(NAME, "pneumaticOn", 1.0);
    private final double pneumaticOff = factory.getConstant(NAME, "pneumaticOff", 0.0);

    /**
     * Instantiates a pneumatic
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    @Inject
    public Pneumatic(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        doublePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 6);
    }

    public PNEUMATIC_STATE getDesiredState() {
        return desiredPneumaticState;
    }

    public void setDesiredState(Pneumatic.PNEUMATIC_STATE desiredPneumaticState) {
        if(robotState.actualCoralArmPivotState == CoralArm.PIVOT_STATE.CLIMB && robotState.actualRampState == Ramp.RAMP_STATE.CLIMB){
            this.desiredPneumaticState = desiredPneumaticState;
            pneumaticOutputsChanged = true;
        }
    }

    public void toggle() {
        if (desiredPneumaticState == PNEUMATIC_STATE.OFF) {
            setDesiredState(PNEUMATIC_STATE.ON);
        } else {
            setDesiredState(PNEUMATIC_STATE.OFF);
        }
    }

    @Override
    public void readFromHardware() {
        if (doublePneumatic.get() == DoubleSolenoid.Value.kReverse) {
            actualPneumaticState = PNEUMATIC_STATE.ON;
        } else {
            actualPneumaticState = PNEUMATIC_STATE.OFF;
        }

//        System.out.println(actualPneumaticState.name());
    }

    @Override
    public void writeToHardware() {
        if (pneumaticOutputsChanged) {
            pneumaticOutputsChanged = false;

            doublePneumatic.set(desiredPneumaticState == PNEUMATIC_STATE.ON ? DoubleSolenoid.Value.kReverse: DoubleSolenoid.Value.kForward);
        }
    }

    @Override
    public void zeroSensors() {
        desiredPneumaticState = PNEUMATIC_STATE.OFF;
        doublePneumatic.set(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void stop() {}

    @Override
    public boolean testSubsystem() {
        return true;
    }

    public enum PNEUMATIC_STATE {
        OFF,
        ON
    }
}
