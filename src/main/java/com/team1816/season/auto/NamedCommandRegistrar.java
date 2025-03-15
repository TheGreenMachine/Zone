package com.team1816.season.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class NamedCommandRegistrar {
    private NamedCommandRegistrar() {
    }

    /**
     * Registers:
     * <ul>
     *     <li>score l1</li>
     *     <li>score l2</li>
     *     <li>score l3</li>
     *     <li>score l4</li>
     *     <br>
     *     <li>ready l1</li>
     *     <li>ready l2</li>
     *     <li>ready l3</li>
     *     <li>ready l4</li>
     *     <li>ready feeder</li>
     *     <br>
     *     <li>score and ready feeder</li>
     *     <li>get feeder coral</li>
     * </ul>
     */
    public static void registerCommands() {
        NamedCommands.registerCommand("score l1", Util.scoreCoralCommand("L1"));
        NamedCommands.registerCommand("score l2", Util.scoreCoralCommand("L2"));
        NamedCommands.registerCommand("score l3", Util.scoreCoralCommand("L3"));
        NamedCommands.registerCommand("score l4", Util.scoreCoralCommand("L4"));

        NamedCommands.registerCommand("ready l1", Util.readyCommand("L1"));
        NamedCommands.registerCommand("ready l2", Util.readyCommand("L2"));
        NamedCommands.registerCommand("ready l3", Util.readyCommand("L3"));
        NamedCommands.registerCommand("ready l4", Util.readyCommand("L4"));
        NamedCommands.registerCommand("ready feeder", Util.readyCommand("FEEDER"));

        NamedCommands.registerCommand("score and ready feeder",
                Commands.waitUntil(Util::isAbleToScore)
                        .andThen(Commands.runOnce(() -> Injector.get(CoralArm.class).setDesiredIntakeState(CoralArm.INTAKE_STATE.OUTTAKE)))
                        .andThen(Commands.waitUntil(() -> Injector.get(RobotState.class).actualCoralArmIntakeState != CoralArm.INTAKE_STATE.OUTTAKE))
                        .andThen(Commands.waitSeconds(.5)) // minor delay just in case
                        .andThen(Commands.runOnce(() -> {
                            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.FEEDER, CoralArm.INTAKE_STATE.INTAKE);
                            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
                        }))
        );

        NamedCommands.registerCommand("get feeder coral",
                Commands.runOnce(() -> {
                            Injector.get(Elevator.class).setDesiredState(Elevator.ELEVATOR_STATE.FEEDER);
                            Injector.get(CoralArm.class).setDesiredState(CoralArm.PIVOT_STATE.FEEDER, CoralArm.INTAKE_STATE.INTAKE);
                        })
                        .andThen(Commands.waitUntil(() -> {
                            CoralArm coralArm = Injector.get(CoralArm.class);
                            return Injector.get(Elevator.class).isElevatorInRange()
                                    && coralArm.isCoralArmIntakeInRange()
                                    && coralArm.isCoralArmPivotInRange();
                        }))
                        .andThen(Commands.waitUntil(() -> Injector.get(CoralArm.class).isBeamBreakTriggered()))
        );
    }

    private static class Util {
        private Util() {
        }

        /**
         * Creates a command to score at a certain level
         */
        public static Command scoreCoralCommand(String level) {
            CoralArm.PIVOT_STATE pivotState = CoralArm.PIVOT_STATE.valueOf(level);
            Elevator.ELEVATOR_STATE elevatorState = Elevator.ELEVATOR_STATE.valueOf(level);

            return Commands.runOnce(() -> {
                        Injector.get(Elevator.class).setDesiredState(elevatorState);
                        Injector.get(CoralArm.class).setDesiredPivotState(pivotState);
                    })
                    .andThen(Commands.waitUntil(Util::isAbleToScore))
                    .andThen(Commands.runOnce(() -> Injector.get(CoralArm.class).setDesiredIntakeState(CoralArm.INTAKE_STATE.OUTTAKE)))
                    .andThen(Commands.waitUntil(() -> Injector.get(RobotState.class).actualCoralArmIntakeState != CoralArm.INTAKE_STATE.OUTTAKE)) // wait until done outtaking)
                    .andThen(Commands.waitSeconds(.5)); // delay to allow coral to fall off intake (if necessary)
        }

        /**
         * Creates a command to score at a certain level
         */
        public static Command readyCommand(String level) {
            CoralArm.PIVOT_STATE pivotState = CoralArm.PIVOT_STATE.valueOf(level);
            Elevator.ELEVATOR_STATE elevatorState = Elevator.ELEVATOR_STATE.valueOf(level);

            return Commands.runOnce(() -> {
                Injector.get(Elevator.class).setDesiredState(elevatorState);
                Injector.get(CoralArm.class).setDesiredPivotState(pivotState);
            });
        }

        public static boolean isAbleToScore() {
            return Injector.get(Elevator.class).isElevatorInRange()
                    && Injector.get(CoralArm.class).isCoralArmPivotInRange();
        }
    }
}
