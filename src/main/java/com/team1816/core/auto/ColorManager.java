package com.team1816.core.auto;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.Color;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A simple manager for selecting the current side of the robot in the simulator
 */
@Singleton
public class ColorManager {
    private final RobotState robotState;

    private final SendableChooser<Color> sideChooser;
    private Color teamColor;

    private boolean changedThisFrame = false; // For usage in other managers that may need to update their stuff based on updates in the SmartDashboard

    @Inject
    public ColorManager(RobotState rs) {
        this.robotState = rs;

        sideChooser = new SendableChooser<>();

        SmartDashboard.putData("Robot color", sideChooser);

        sideChooser.setDefaultOption(Color.BLUE.name(), Color.BLUE); // initialize options
        sideChooser.addOption(Color.RED.name(), Color.RED); // initialize options
    }

    /**
     * Updates the choosers in realtime
     *
     * @return true if updated
     */
    public boolean update() {
        Color selectedColor = getSelectedColor();

        changedThisFrame = teamColor != selectedColor;

        // if auto has been changed, update selected auto mode + thread
        if (changedThisFrame) {
            GreenLogger.log(
                    "Color changed from: " + teamColor + ", to: " + selectedColor
            );
            teamColor = selectedColor;
            robotState.allianceColor = teamColor;
        }

        return changedThisFrame;
    }

    public void outputToSmartDashboard() {
        if (teamColor != null) {
            SmartDashboard.putString("RobotColorSelected", teamColor.name());
        }
    }

    /**
     * Returns the selected color if the robot is in sim, and the driver station color if
     * the robot is real
     */
    public Color getTrueRobotColor() {
        if (RobotBase.isSimulation()) {
            return getSelectedColor();
        } else if (RobotBase.isReal()) {
            var dsAlliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : sideChooser.getSelected(); //ternary hell
            return (dsAlliance == DriverStation.Alliance.Red) ? Color.RED : Color.BLUE;
        }

        GreenLogger.log("Robot is in an invalid state, defaulting to blue alliance color!");

        return Color.BLUE;
    }

    public Color getSelectedColor() {
        return sideChooser.getSelected();
    }

    public boolean isColorChanged() {
        return changedThisFrame;
    }
}
