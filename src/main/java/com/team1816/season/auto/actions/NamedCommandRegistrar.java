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

        NamedCommands.registerCommand("ramp score l1 deep", Commands.runOnce(() -> {
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.SCORE_DEEP);
        }));

        NamedCommands.registerCommand("ramp hold l1", Commands.runOnce(() -> {
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.L1_FEEDER);
        }));

        NamedCommands.registerCommand("ramp hold l234", Commands.runOnce(() -> {
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.L234_FEEDER);
        }));

        NamedCommands.registerCommand("coral feeder", Commands.runOnce(() -> {
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.L234_FEEDER);
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.FEEDER, CoralArm.INTAKE_STATE.INTAKE);
        }));

        NamedCommands.registerCommand("coral l2", Commands.runOnce(() -> {
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.L2_CORAL);
            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.L2_CORAL, CoralArm.INTAKE_STATE.HOLD);
        }));

        NamedCommands.registerCommand("coral l3", Commands.runOnce(() -> {
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.L3_CORAL);
            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.L3_CORAL, CoralArm.INTAKE_STATE.HOLD);
        }));

        NamedCommands.registerCommand("coral l4", Commands.runOnce(() -> {
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.L4);
            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.L4, CoralArm.INTAKE_STATE.HOLD);
        }));

        NamedCommands.registerCommand("coral outtake", Commands.runOnce(() -> {
            Injector.get(CoralArm.class).setDesiredIntakeState(CoralArm.INTAKE_STATE.OUTTAKE);
        }));

        NamedCommands.registerCommand("coral l234 initialize", Commands.runOnce(() -> {
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.FEEDER, CoralArm.INTAKE_STATE.HOLD);
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.L234_FEEDER);
        }));

        NamedCommands.registerCommand("coral l1 initialize", Commands.runOnce(() -> {
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.FEEDER, CoralArm.INTAKE_STATE.HOLD);
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.L1_FEEDER);
        }));

        NamedCommands.registerCommand("l2 algae", Commands.runOnce(() -> {
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.L2_ALGAE);
            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.L2_ALGAE, CoralArm.INTAKE_STATE.REMOVE_ALGAE);
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.L234_FEEDER);
        }));


        NamedCommands.registerCommand("l3 algae", Commands.runOnce(() -> {
            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.L3_ALGAE);
            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.L3_ALGAE, CoralArm.INTAKE_STATE.REMOVE_ALGAE);
            Injector.get(Ramp.class).setDesiredState(Ramp.RAMP_STATE.L234_FEEDER);
        }));
    }
}
