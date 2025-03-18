package com.team1816.season.auto.actions;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.lib.Injector;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import com.team1816.season.subsystems.Ramp;
import edu.wpi.first.wpilibj2.command.Commands;

public final class NamedCommandRegistrar {
    private NamedCommandRegistrar() {
    }

    /**
     * Registers:
     * <ul>
     *     <li>ramp score l1</li>
     *     <li>ramp hold l1</li>
     * </ul>
     */
    public static void registerCommands() {
        NamedCommands.registerCommand("ramp score l1", Commands.runOnce(() -> {
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.SCORE);
        }));

        NamedCommands.registerCommand("ramp hold l1", Commands.runOnce(() -> {
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.L1_FEEDER);
        }));

        NamedCommands.registerCommand("ramp hold l234", Commands.runOnce(() -> {
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.L234_FEEDER);
        }));

        NamedCommands.registerCommand("coral feeder", Commands.runOnce(() -> {
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
        }));

        NamedCommands.registerCommand("coral l4", Commands.runOnce(() -> {
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.L4);
            System.out.println("moved to l4 position");
        }));

        NamedCommands.registerCommand("coral outtake", Commands.runOnce(() -> {
            Injector.get(CoralArm.class).setDesiredIntakeState(CoralArm.INTAKE_STATE.OUTTAKE);
        }));
    }
}
