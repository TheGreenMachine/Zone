# PathPlanner Guide

A guide on how to use PathPlanner in our code!

For a guide on using PathPlanner GUI, go to <a href='https://pathplanner.dev/pathplanner-gui.html'>Official PathPlanner GUI Guide</a>

## Adding an auto

Adding a PathPlanner auto to the code is very simple. Navigate to `AutoModeManager` and find the
`DesiredAuto` enum. Just drop in a supplier for a `PathPlannerAutoMode` with a descriptive name for
the enum value like so:

```java
enum DesiredAuto {
    // ...
    
    MIDDLE_SCORE_2(() -> new PathPlannerAutoMode("Middle Score 2 Auto")),
    
    // ...
}
```

It will automatically grab the specified auto that PathPlanner generated and create it.

To mirror the auto (in case you created an auto from the top of the field and want to reflect it to
the bottom of the field), just set the `mirror` parameter to true in the `PathPlannerAutoMode`
constructor:

```java
new PathPlannerAutoMode("Top Score 1", true)
```

## Programming Named Commands

To make creating autos as easy as possible, we use PathPlanner to make the robot do most of its
actions. To do this, we need to use something called Named Commands. These allow us to interact
with subsystems indirectly using PathPlanner, and create autos with maximal ease.

So how do we implement Named Commands? Well, there's two locations we do this in:
- The subsystem itself
- The Named Command registrar

For organisation, commands that only interact with a single subsystem should be put in the
implementNamedCommands() method (in the subsystem), and commands that interact with multiple
subsystems should be put  in the Named Commands registrar. Here's some basic examples for this
convention:

| Subsystem                                                         | Named Command Registrar                                                          |
|-------------------------------------------------------------------|----------------------------------------------------------------------------------|
| Telling a collector to go into INTAKE mode                        | A multi-step action where an elevator is raised and the collector on it outtakes |
| A robotic arm making a grabbing motion                            | Two subsystems are brought into a certain position in parallel                   |
| A collector goes into INTAKE mode until a beam break is triggered | The robot latches onto a monkey bar and activates a pneumatic to climb           |

This allows us to maintain code cleanliness and preventing the headache of scrolling through
hundreds of lines of handlers for events (similar to the problem with our input handlers).

## Registering named commands

A simple registration of a Named Command might look something like this:

```java
NamedCommands.registerCommand("do a thing", Commands.runOnce(() -> doAThing()));
```

## Defining Named Commands inside subsystems

Again, this should only be done when the named command only interacts with **one subsystem**.

Every subsystem extends from the Subsystem abstract class, which contains an optional method that
implements named commands. If the method does not already exist, simply override the superclass
method and register named commands directly in that method:

```java
/**
 * Creates event triggers for:
 * <ul>
 *     <li>collector intake</li>
 *     <li>collector outtake</li>
 *     <li>collector hold</li>
 * </ul>
 */
@Override
public void implementNamedCommands() {
    NamedCommands.registerCommand(NAME + " intake", Commands.runOnce(
            () -> setDesiredIntakeState(INTAKE_MODE.INTAKE)
    ));
    
    // ...
}
```

*`NAME` is the constant defining the name of the subsystem*

Make sure to document these commands! You should leave a JavaDoc comment with all the commands that
the method implements, as shown above.

Additionally, it's good practice to follow the naming conventions we use: `[name of subsystem]
[name of action]`. The name of the action can contain spaces.

## Defining Named Commands in the Named Command Registrar

The process to define Named Commands in the registrar is relatively similar to defining Named
Commands in subsystems.

```java
/**
 * Creates event triggers for:
 * <ul>
 *     <li>...</li>
 *     <li>aim and shoot</li>
 *     <li>...</li>
 * </ul>
 */
public static void registerCommands() {
    // ...

    // aims the aimer, waiting for it to get into position, and shoots after half a second
    NamedCommands.registerCommand("aim and shoot", Commands.runOnce(
            () -> Injector.get(Aimer.class).setDesiredAimState(AIM_STATE.AIMED))
                    .alongWith(
                            Commands.waitUntil(() -> Injector.get(Aimer.class).isAimed())
                                    .andThen(Commands.waitSeconds(.5))
                                    .andThen(() -> Injector.get(Shooter.class).shoot())
                                    .andThen(Commands.waitUntil(() -> Injector.get(Shooter.class).isDoneShooting()))
                    ));

    // ...
}
```

Most Named Commands you create will be in the registrar, so if there's too many of them in the
`registerCommands()` method it's a good idea to split it up into multiple methods and call them in
`registerCommands()`.

Unfortunately, due to the way the robot is programmed, we have to use the Injector multiple times
per second to wait for something to finish, lest we break some Java laws and store the references
somewhere. Maybe in the future, we'll design a wrapper for PathPlanner's Named Command system that
rectifies this issue.

## Some final notes

### Asynchronous operation

Some subsystems can do things while the robot is driving. In an auto where this is necessary,
you should set the desired state of the subsystem BEFORE the robot starts following a path.

It can be dangerous to do just this, however. If an operation needs to be done that requires the
desired state to be the actual, physical state of the robot (ex. an aiming operation needing to be
in position before another subsystem shoots a game piece), there can be times when the robot
finishes a path before the subsystem is finished with whatever it needs to do. The solution is to
put a `Commands.waitUntil(/* subsystem is finished */)` and supply a boolean that states whether the
operation is finished or not, before an `andThen` call that actually does the operation.

For an example of the code, the code snippet for **Defining Named Commands in the Named Command
Registrar** does exactly this (as well as aiming the aimer as a failsafe for if PathPlanner didn't
work for some reason).

In cases where you want to set a delay before you set the desired state of the subsystem, create a
**Parallel Command Group** and put the follow call in there, as well as a sequential command group.
In the sequential command group, put a Wait call and call the command afterward.
