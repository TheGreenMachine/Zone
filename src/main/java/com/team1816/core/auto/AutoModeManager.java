package com.team1816.core.auto;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.modes.AutoMode;
//import com.team1816.lib.auto.modes.AutopathMode;
import com.team1816.lib.auto.modes.DefaultMode;
import com.team1816.lib.auto.modes.DriveStraightMode;
//import com.team1816.lib.autopath.Autopath;
import com.team1816.lib.auto.modes.TuneDrivetrainMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.modes.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An integrated and optimized manager for autonomous mode selection and configuration
 */
@Singleton
public class AutoModeManager {

    /**
     * Properties: Selection
     */
    private RobotState robotState;
    private final SendableChooser<DesiredAuto> autoModeChooser;
    private final SendableChooser<Color> sideChooser;
    private DesiredAuto desiredAuto;
    private Color teamColor;

    /**
     * Properties: Dynamic Auto
     */

    /**
     * Properties: Execution
     */
    private AutoMode autoMode = new DriveStraightMode();
    private static Thread autoModeThread;

    /**
     * Instantiates and AutoModeManager with a default option and selective computation
     *
     * @param rs RobotState
     */
    @Inject
    public AutoModeManager(RobotState rs) {
        robotState = rs;
        autoModeChooser = new SendableChooser<>(); // Shuffleboard dropdown menu to choose desired auto mode
        sideChooser = new SendableChooser<>(); // Shuffleboard dropdown menu to choose desired side / bumper color

        SmartDashboard.putData("Auto mode", autoModeChooser); // appends chooser to shuffleboard=

        for (DesiredAuto desiredAuto : DesiredAuto.values()) {
            autoModeChooser.addOption(desiredAuto.name(), desiredAuto);
        }
        autoModeChooser.setDefaultOption(
            DesiredAuto.DEFAULT.name(),
            DesiredAuto.DEFAULT
        );

        SmartDashboard.putData("Robot color", sideChooser); // appends chooser to shuffleboard

        sideChooser.setDefaultOption(Color.BLUE.name(), Color.BLUE); // initialize options
        sideChooser.addOption(Color.RED.name(), Color.RED); // initialize options

        /**
         * Dynamic Auto
         */

        reset();
    }

    /**
     * Resets properties to default and resets the thread
     */
    public void reset() {
        autoMode = new DriveStraightMode();
        autoModeThread = new Thread(autoMode::run);
        desiredAuto = DesiredAuto.DRIVE_STRAIGHT;
        teamColor = sideChooser.getSelected();
        robotState.allianceColor = teamColor;
    }

    /**
     * Updates the choosers in realtime
     *
     * @return true if updated
     */
    public boolean update() {
        DesiredAuto selectedAuto = autoModeChooser.getSelected();

        Color selectedColor = Color.BLUE;

        if (RobotBase.isSimulation()) {
            selectedColor = sideChooser.getSelected();
        } else if (RobotBase.isReal()) {
            var dsAlliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : sideChooser.getSelected(); //ternary hell
            selectedColor = (dsAlliance == DriverStation.Alliance.Red) ? Color.RED : Color.BLUE;
        }

        boolean autoChanged = desiredAuto != selectedAuto;
        boolean startPosChanged = false;
        boolean colorChanged = teamColor != selectedColor;

        // if auto has been changed, update selected auto mode + thread
        if (autoChanged || colorChanged || startPosChanged) {
            if (autoChanged) {
                desiredAuto = selectedAuto;
                GreenLogger.log(
                    "Auto changed from: " + desiredAuto + ", to: " + selectedAuto.name()
                );
            }
            if (colorChanged) {
                teamColor = selectedColor;
                GreenLogger.log("Robot color changed from: " + teamColor + ", to: " + selectedColor);
            }

            autoMode = generateAutoMode(selectedAuto, selectedColor);
            autoModeThread = new Thread(autoMode::run);
        }
        robotState.allianceColor = teamColor;

        return autoChanged || colorChanged;
    }

    public void updateAutoMode(){
        autoMode = generateAutoMode(autoModeChooser.getSelected(), getSelectedColor());
        autoModeThread = new Thread(autoMode::run);
    }

    /**
     * Outputs values to SmartDashboard
     */
    public void outputToSmartDashboard() {
        if (desiredAuto != null) {
            SmartDashboard.putString("AutoModeSelected", desiredAuto.name());
        }
        if (teamColor != null) {
            SmartDashboard.putString("RobotColorSelected", teamColor.name());
        }
    }

    /**
     * Returns the selected autonomous mode
     *
     * @return AutoMode
     * @see AutoMode
     */
    public AutoMode getSelectedAuto() {
        return autoMode;
    }

    /**
     * Returns the selected color
     *
     * @return Color
     * @see Color
     */
    public Color getSelectedColor() {
        return sideChooser.getSelected();
    }

    /**
     * Executes the auto mode and respective thread
     */
    public void startAuto() {
        autoModeThread.start();
    }

    /**
     * Stops the auto mode
     */
    public void stopAuto() {
        if (autoMode != null) {
            autoMode.stop();
            autoModeThread = new Thread(autoMode::run);
        }
    }

    /**
     * Enum for AutoModes
     */
    enum DesiredAuto {
        DEFAULT,

        DRIVE_STRAIGHT,

        TUNE_DRIVETRAIN,

//        AUTOPATH,

        TOP_SIDE_1_SCORE_1,

        TOP_SIDE_1_SCORE_1_FEEDER,

//        TOP_SIDE_1_SCORE_2,

        MIDDLE_SIDE_2_SCORE_1,

        MIDDLE_SIDE_2_SCORE_1_TOP_FEEDER,

        MIDDLE_SIDE_2_SCORE_1_BOTTOM_FEEDER,

//        MIDDLE_SIDE_3_SCORE_2,

        BOTTOM_SIDE_3_SCORE_1,

        BOTTOM_SIDE_3_SCORE_1_FEEDER,

//        BOTTOM_SIDE_3_SCORE_2,

        TOP_DRIVE_STRAIGHT,

        MIDDLE_DRIVE_STRAIGHT,

        BOTTOM_DRIVE_STRAIGHT,

//        DYNAMIC_TRAJECTORY_ONLY,

//        DYNAMIC_PLACE_1,
//
//        DYNAMIC_PLACE_2,
//
//        DYNAMIC_PLACE_3

//        TEST_DYNAMIC_PATHS
        }


    /**
     * Generates each AutoMode by demand
     *
     * @param mode desiredMode
     * @return AutoMode
     * @see AutoMode
     */
    private AutoMode generateAutoMode(DesiredAuto mode, Color color) {
        switch (mode) {
            case DEFAULT:
                robotState.dIsAutoDynamic = false;
                return new DefaultMode();
            case DRIVE_STRAIGHT:
                robotState.dIsAutoDynamic = false;
                return new DriveStraightMode();
            case TUNE_DRIVETRAIN:
                robotState.dIsAutoDynamic = false;
                return new TuneDrivetrainMode();
//            case AUTOPATH:
//                robotState.dIsAutoDynamic = false;
//                return new AutopathMode();
            case MIDDLE_SIDE_2_SCORE_1:
                robotState.dIsAutoDynamic = false;
                return new MiddlePlace1AutoMode(color, MiddlePlace1AutoMode.ENDING_FEEDER.NONE);
            case MIDDLE_SIDE_2_SCORE_1_TOP_FEEDER:
                robotState.dIsAutoDynamic = false;
                return new MiddlePlace1AutoMode(color, MiddlePlace1AutoMode.ENDING_FEEDER.TOP);
            case MIDDLE_SIDE_2_SCORE_1_BOTTOM_FEEDER:
                robotState.dIsAutoDynamic = false;
                return new MiddlePlace1AutoMode(color, MiddlePlace1AutoMode.ENDING_FEEDER.BOTTOM);
//            case MIDDLE_SIDE_3_SCORE_2:
//                robotState.dIsAutoDynamic = false;
//                return new MiddlePlace2AutoMode(color);
            case TOP_DRIVE_STRAIGHT:
                robotState.dIsAutoDynamic = false;
                return new DriveOffLineTopAutoMode(color);
            case MIDDLE_DRIVE_STRAIGHT:
                robotState.dIsAutoDynamic = false;
                return new DriveOffLineMiddleAutoMode(color);
            case BOTTOM_DRIVE_STRAIGHT:
                robotState.dIsAutoDynamic = false;
                return new DriveOffLineBottomAutoMode(color);
            case BOTTOM_SIDE_3_SCORE_1:
                robotState.dIsAutoDynamic = false;
                return new BottomPlace1AutoMode(color, false);
            case BOTTOM_SIDE_3_SCORE_1_FEEDER:
                robotState.dIsAutoDynamic = false;
                return new BottomPlace1AutoMode(color, true);
//            case BOTTOM_SIDE_3_SCORE_2:
//                robotState.dIsAutoDynamic = false;
//                return new BottomPlace2AutoMode(color);
//            case DYNAMIC_PLACE_1:
//                robotState.dIsAutoDynamic = true;
//                return new DynamicPlace1();
//            case DYNAMIC_PLACE_2:
//                robotState.dIsAutoDynamic = true;
//                return new DynamicPlace2();
//            case DYNAMIC_PLACE_3:
//                robotState.dIsAutoDynamic = true;
//                return new DynamicPlace3();
            case TOP_SIDE_1_SCORE_1:
                robotState.dIsAutoDynamic = false;
                return  new TopPlace1AutoMode(color, false);
            case TOP_SIDE_1_SCORE_1_FEEDER:
                robotState.dIsAutoDynamic = false;
                return  new TopPlace1AutoMode(color, true);
//            case TOP_SIDE_1_SCORE_2:
//                robotState.dIsAutoDynamic = false;
//                return new TopPlace2AutoMode(color);
//            case DYNAMIC_TRAJECTORY_ONLY:
//                robotState.isAutoDynamic = true;
//                RobotState.dynamicAutoChanged = true;
//                return new DynamicTrajectoryOnlyAutoMode(robotState);
//            case TEST_DYNAMIC_PATHS:
//                return new TestAllDynamicPointsAutoMode();
            default:
            robotState.dIsAutoDynamic = false;
                GreenLogger.log("Defaulting to DefaultMode");
                return new DefaultMode();
        }
    }
}
