package com.team1816.season.subsystems;

import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.pcm.ISolenoid;

@Singleton
public class Pneumatic extends Subsystem {
    /**
     * Properties
     */
    private static final String NAME = "pneumatic";

    /**
     * Components
     */
    e;
    private final ISolenoid pneumatic;

    /**
     * State
     */
    private Pneumatic.PNEUMATIC_STATE desiredPneumaticState = PNEUMATIC_STATE.OFF;
    private Pneumatic.PNEUMATIC_STATE actualPneumaticState = PNEUMATIC_STATE.OFF;

    private boolean pneumaticOutputsChanged = false;

    private double desiredPneumaticPosition = 0.0;
    private double actualPneumaticPosition = 0.0;

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
        pneumatic = factory.getSolenoid(NAME, "pneumatic");

        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME);
    }

    public void setDesiredState(Pneumatic.PNEUMATIC_STATE desiredPneumaticState) {
        this.desiredPneumaticState = desiredPneumaticState;
        pneumaticOutputsChanged = true;
    }

    public void changePneumatic() {
        if (desiredPneumaticState == PNEUMATIC_STATE.OFF) {
            desiredPneumaticState = PNEUMATIC_STATE.ON;
        } else {
            desiredPneumaticState = PNEUMATIC_STATE.OFF;
        }
        pneumatic.set(desiredPneumaticState == PNEUMATIC_STATE.ON);
        actualPneumaticState = desiredPneumaticState;
    }

    @Override
    public void readFromHardware() {
        if (pneumatic.get()) {
            actualPneumaticState = PNEUMATIC_STATE.ON;
        } else {
            actualPneumaticState = PNEUMATIC_STATE.OFF;
        }
    }

    @Override
    public void writeToHardware() {
        if (pneumaticOutputsChanged) {
            pneumaticOutputsChanged = false;
            switch (desiredPneumaticState) {
                case ON -> {
                    desiredPneumaticPosition = pneumaticOn;
                }
                case OFF -> {
                    desiredPneumaticPosition = pneumaticOff;
                }
            }
            pneumatic.set(desiredPneumaticState == PNEUMATIC_STATE.ON);
            actualPneumaticState = desiredPneumaticState;
        }
    }

    @Override
    public void zeroSensors() {
        desiredPneumaticState = PNEUMATIC_STATE.OFF;
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
