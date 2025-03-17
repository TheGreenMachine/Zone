package com.team1816.core.auto;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.states.RobotState;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.auto.modes.NoopAutoMode;
import com.team1816.lib.auto.modes.PathPlannerAutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;
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
     * Properties: Execution
     */
    private AutoMode autoMode = DesiredAuto.DRIVE_STRAIGHT.autoMode;
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
        
        SmartDashboard.putData("Auto mode", autoModeChooser); // appends chooser to shuffleboard=
        sideChooser = new SendableChooser<>(); // Shuffleboard dropdown menu to choose desired side / bumper color
        
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
        
        reset();
    }
    
    /**
     * Resets properties to default and resets the thread
     */
    public void reset() {
        autoMode = DesiredAuto.DEFAULT.autoMode;
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
            
            autoMode = selectedAuto.autoMode;
            autoModeThread = new Thread(autoMode::run);
        }
        robotState.allianceColor = teamColor;
        
        return autoChanged || colorChanged;
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
        DEFAULT(new NoopAutoMode()),
        
        DRIVE_STRAIGHT(new PathPlannerAutoMode("Drive Straight Auto")),
        
        TOP_3L1(new PathPlannerAutoMode("Top 3L1 Auto")),
        BOTTOM_3L1(new PathPlannerAutoMode("Top 3L1 Auto", true)),
        
        OPTIMIZED_TOP_4L1(new PathPlannerAutoMode("Optimized Top 4L1 Auto")),
        OPTIMIZED_BOTTOM_4L1(new PathPlannerAutoMode("Optimized Top 4L1 Auto", true)),
        
        MIDDLE_1L4(new PathPlannerAutoMode("Middle 1L4 Auto")),
        
        TOP_4L4(new PathPlannerAutoMode("Top 4L4 Auto")),
        BOTTOM_4L4(new PathPlannerAutoMode("Top 4L4 Auto", true)),
        
        ;
        
        DesiredAuto(AutoMode autoMode) {
            this.autoMode = autoMode;
        }
        
        public final AutoMode autoMode;
    }
}
