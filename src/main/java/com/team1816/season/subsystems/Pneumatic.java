package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.Timer;

/**
 * Subsystem that models a "windmill" style climber
 */
@Singleton
public class Pneumatic extends Subsystem {

    /**
     * Properties
     */
    private static final String NAME = "climber";

    /**
     * Components
     */
    private final ISolenoid pneumatic;

    /**
     * State
     */
    private boolean pneumaticOff;
    private boolean needsPneumatic = false;
    private boolean pneumaticOn = false;

    /**
     * Instantiates a climber from base subsystem properties
     * @param inf Infrastructure
     * @param rs RobotState
     */
    @Inject
    public Pneumatic(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        pneumatic = factory.getSolenoid(NAME, "pneumatic");

        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME);

        pneumaticOff = false;
    }

    /**
     * Unlocks the climber
     */
    public void unlock() {
        System.out.println("Unlocking Climber!");
        pneumaticOff = true;
    }

    /**
     * Toggles the top clamp
     */
    public void setPneumatic() {
        if (!pneumaticOff) {
            System.out.println("Pneumatic On");
            return;
        }
        pneumaticOn = !pneumaticOn;
        needsPneumatic = true;
    }

    /**
     * Sets the top and bottom clamps based on desired states and order
     * @param pneumaticOn boolean
     */
    private void setPneumatics(boolean pneumaticOn) {
        if (needsPneumatic) {
            needsPneumatic = false;
            pneumatic.set(pneumaticOn);
            System.out.println("setting climber clamps!");
            Timer.delay(.25);
        }
    }

    /** Periodic */

    /**
     * Reads from motor values and updates state
     */
    @Override
    public void readFromHardware() {}

    /**
     * Writes outputs to the motor based on controlMode and desired state
     */
    @Override
    public void writeToHardware() {
        setPneumatics(robotState.pneumaticOn);
    }

    /** Config and Tests */

    /**
     * Zeroes the climber motor position
     */
    @Override
    public void zeroSensors() {
        pneumaticOff = false;
        needsPneumatic = false;
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void stop() {}

    /**
     * Tests the subsystem
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        return true;
    }

    /** Modes and Stages */

    /**
     * Base static class for climber stages
     */
    static class Stage {

        public final boolean pneumaticOn;

        Stage(
                boolean pneumaticOn
        ) {
            this.pneumaticOn = pneumaticOn;
        }

        Stage() {
            this(false);
        }
    }
}
