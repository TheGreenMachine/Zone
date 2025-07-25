package com.team1816.lib.hardware.factory;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.google.common.io.Resources;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.*;
import com.team1816.lib.hardware.components.gyro.GhostPigeonIMU;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.hardware.components.gyro.Pigeon2Impl;
import com.team1816.lib.hardware.components.gyro.PigeonIMUImpl;
import com.team1816.lib.hardware.components.ledManager.CANdleImpl;
import com.team1816.lib.hardware.components.ledManager.CanifierImpl;
import com.team1816.lib.hardware.components.ledManager.GhostLEDManager;
import com.team1816.lib.hardware.components.ledManager.ILEDManager;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.*;
import com.team1816.lib.hardware.components.sensor.GhostProximitySensor;
import com.team1816.lib.hardware.components.sensor.IProximitySensor;
import com.team1816.lib.hardware.components.sensor.ProximitySensor;
import com.team1816.lib.subsystems.drive.SwerveModule;
import com.team1816.lib.util.driveUtil.DriveConversions;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.TunerConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;

import javax.annotation.Nonnull;
import java.net.URL;
import java.nio.charset.Charset;
import java.util.Map;
import java.util.Objects;

/**
 * This class employs the MotorFactory and SensorFactory with yaml integrations and is the initial entry point to
 * create and initialize any and all components on a robot.
 *
 * @see MotorFactory
 * @see SensorFactory
 * @see YamlConfig
 */
@Singleton
public class RobotFactory {

    private RobotConfiguration config;

    public RobotFactory() {
        var robotName = System.getenv("ROBOT_NAME");
        if (robotName == null) {
            robotName = "default";
            DriverStation.reportWarning(
                    "ROBOT_NAME environment variable not defined, falling back to default.config.yml!",
                    false
            );
        }
        robotName = robotName.toLowerCase();
        System.out.println("Loading Config for " + robotName);
        try {
            config =
                    YamlConfig.loadFrom(
                            this.getClass()
                                    .getClassLoader()
                                    .getResourceAsStream("yaml/" + robotName + ".config.yml")
                    );
        } catch (Exception e) {
            DriverStation.reportError("Yaml Config error!", e.getStackTrace());
        }
    }

    /**
     * This method reads a resource file called 'git_hash.txt'.
     *
     * The resource contains the git hash for the current version of the repository
     * you're on.
     *
     * @return a string representation of the current git hash
     */
    public static String getGitHash() {
        String gitHashStr;
        try {
            URL input = Resources.getResource("git_hash");

            if (input == null) {
                gitHashStr = "UNABLE TO FIND THE GIT HASH.";
            } else {
                gitHashStr = Resources.toString(input, Charset.defaultCharset());
            }
        } catch (Exception e) {
            GreenLogger.log("Exception occurred: " + e.toString());
            gitHashStr = "NO VALID GIT HASH FOUND";
        }

        GreenLogger.log("Git Hash: " + gitHashStr);

        return gitHashStr;
    }

    public IGreenMotor getMotor(
            String subsystemName,
            String name,
            Map<String, PIDSlotConfiguration> pidConfigs,
            int remoteSensorId
    ) {

        IGreenMotor motor = null;
        var subsystem = getSubsystem(subsystemName);


        // Identifying motor
        if (subsystem.implemented) {
            if (isMotorValid(subsystem.motors, name)) {
                String canBus = subsystem.motors.get(name).isLowSpeedCanBus != null && subsystem.motors.get(name).isLowSpeedCanBus ? "roboRio" : config.infrastructure.canBusName;
                switch (subsystem.motors.get(name).motorType) {
                    case TalonFX -> {
                        motor =
                                MotorFactory.createDefaultTalon(
                                        subsystem.motors.get(name).id,
                                        name,
                                        true,
                                        subsystem,
                                        pidConfigs,
                                        remoteSensorId,
                                        canBus
                                );
                    }
                    case TalonSRX -> {
                        motor =
                                MotorFactory.createDefaultTalon(
                                        subsystem.motors.get(name).id,
                                        name,
                                        false,
                                        subsystem,
                                        pidConfigs,
                                        remoteSensorId,
                                        config.infrastructure.canBusName
                                );
                    }
                    case SparkMax -> {
                        motor =
                            MotorFactory.createSpark(
                                subsystem.motors.get(name).id,
                                name,
                                subsystem,
                                pidConfigs
                            );
                    }
                    case VictorSPX -> {
                        GreenLogger.log("Victors cannot be main!");
                    }
                }
            }
            // Never make the victor a main
        }

        // report creation of motor
        if (motor == null) {
            reportGhostWarning("Motor", subsystemName, name);
            motor =
                    MotorFactory.createGhostMotor(
                            (int) (getConstant(subsystemName, "maxVelOpenLoop", 1, false)),
                            0,
                            name,
                            subsystem
                    );
        } else {
            GreenLogger.log(
                    "Created " +
                            motor.getClass().getSimpleName() +
                            " id:" +
                            motor.getDeviceID()
            );
        }

        return motor;
    }

    public IGreenMotor getMotor(String subsystemName, String name) {
        return getMotor(subsystemName, name, getSubsystem(subsystemName).pidConfig, -1);
    }

    public IGreenMotor getFollowerMotor(
            String subsystemName,
            String name,
            IGreenMotor main,
            boolean opposeLeaderDirection
    ) {
        IGreenMotor followerMotor = null;
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented && main != null) {
            if (isMotorValid(subsystem.motors, name)) {
                switch(subsystem.motors.get(name).motorType) {
                    case TalonFX -> {
                        followerMotor =
                                MotorFactory.createFollowerTalon(
                                        subsystem.motors.get(name).id,
                                        name,
                                        true,
                                        main,
                                        subsystem,
                                        subsystem.pidConfig,
                                        config.infrastructure.canBusName,
                                        opposeLeaderDirection
                                );
                    }
                    case TalonSRX -> {
                        followerMotor =
                                MotorFactory.createFollowerTalon(
                                        subsystem.motors.get(name).id,
                                        name,
                                        false,
                                        main,
                                        subsystem,
                                        subsystem.pidConfig,
                                        config.infrastructure.canBusName,
                                        opposeLeaderDirection
                                );
                    }
                    case SparkMax -> {
                        MotorFactory.createFollowerSpark(
                            subsystem.motors.get(name).id,
                            name,
                            subsystem,
                            subsystem.pidConfig,
                            main,
                            opposeLeaderDirection
                        );
                    }
                    case VictorSPX -> {
                        followerMotor =
                                MotorFactory.createFollowerVictor(
                                        subsystem.motors.get(name).id,
                                        name,
                                        main,
                                        subsystem,
                                        subsystem.pidConfig
                                );
                    }
                }
            }
        }
        if (followerMotor == null) {
            if (subsystem.implemented) reportGhostWarning("Motor", subsystemName, name);
            followerMotor =
                    MotorFactory.createGhostMotor(
                            (int) getConstant(subsystemName, "maxVelOpenLoop", 1),
                            0,
                            name,
                            subsystem
                    );
        }
        if (main != null) {
            followerMotor.setInvertedMotor(main.getInvertedMotor());
        }
        return followerMotor;
    }

    public LegacySwerveModuleConstants getCTRESwerveModule(String subsystemName, String name) {
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> currentModule = null;

        switch(name){
            case "backRight":
                currentModule = TunerConstants.BackRight;
                break;
            case "frontRight":
                currentModule = TunerConstants.FrontRight;
                break;
            case "frontLeft":
                currentModule = TunerConstants.FrontLeft;
                break;
            case "backLeft":
                currentModule = TunerConstants.BackLeft;
                break;
        }

        var moduleConfig = new LegacySwerveModuleConstants()
                // General Drivetrain
                .withSpeedAt12VoltsMps(currentModule.SpeedAt12Volts)
                .withFeedbackSource(LegacySwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                // CANCoder
                .withCANcoderId(currentModule.EncoderId)
                .withCANcoderOffset(currentModule.EncoderOffset)
                // General Motor
                .withCouplingGearRatio(currentModule.CouplingGearRatio)
                .withWheelRadius(Units.metersToInches(currentModule.WheelRadius))
                .withLocationX(currentModule.LocationX) //IMPORTANT: IF THIS IS NOT A SQUARE SWERVEDRIVE, THESE MUST BE DIFFERENT.
                .withLocationY(currentModule.LocationY)
                // Drive Motor
                .withDriveMotorClosedLoopOutput(LegacySwerveModule.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(currentModule.DriveMotorGains)
                .withDriveMotorId(currentModule.DriveMotorId)
                .withSlipCurrent(currentModule.SlipCurrent)
                .withDriveMotorGearRatio(currentModule.DriveMotorGearRatio)
                .withDriveMotorInverted(currentModule.DriveMotorInverted)
                // Azimuth Motor
                .withSteerMotorClosedLoopOutput(LegacySwerveModule.ClosedLoopOutputType.Voltage)
                .withSteerMotorGains(currentModule.SteerMotorGains)
                .withSteerMotorId(currentModule.SteerMotorId)
                .withSteerMotorGearRatio(currentModule.SteerMotorGearRatio)
                .withSteerMotorInverted(currentModule.SteerMotorInverted)
                .withSteerFrictionVoltage(currentModule.SteerFrictionVoltage)
                .withDriveFrictionVoltage(currentModule.DriveFrictionVoltage)
                .withSteerMotorInitialConfigs(currentModule.SteerMotorInitialConfigs)
                .withDriveMotorInitialConfigs(currentModule.DriveMotorInitialConfigs)
                ;

        return moduleConfig;
    }

    public SwerveModule getSwerveModule(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        ModuleConfiguration module = subsystem.swerveModules.modules.get(name);
        if (module == null) {
            DriverStation.reportError(
                    "No swerve module with name " + name + " subsystem " + subsystemName,
                    true
            );
            return null;
        }

        var moduleConfig = new SwerveModule.ModuleConfig();
        moduleConfig.moduleName = name;
        moduleConfig.azimuthMotorName = module.azimuth; // getAzimuth and drive give ID I think - not the module name (ex: leftRear)
        moduleConfig.azimuthPid =
                getPidSlotConfig(subsystemName, "slot0", PIDConfig.Azimuth);
        moduleConfig.driveMotorName = module.drive;
        moduleConfig.drivePid = getPidSlotConfig(subsystemName, "slot0", PIDConfig.Drive);
        moduleConfig.azimuthEncoderHomeOffset = module.constants.get("encoderOffset");

        var canCoder = getModuleCanCoder(subsystemName, name);

        return new SwerveModule(subsystemName, moduleConfig, canCoder);
    }

    public CANcoder getModuleCanCoder(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        var module = subsystem.swerveModules.modules.get(name);
        CANcoder canCoder = null;
        if (
                module != null &&
                        module.canCoder != null &&
                        subsystem.canCoders.get(module.canCoder) >= 0
        ) {
            canCoder =
                    MotorFactory.createCanCoder(
                            subsystem.canCoders.get(module.canCoder),
                            config.infrastructure.canBusName,
                            subsystem.canCoders.get(subsystem.invertCanCoder) != null &&
                                    subsystem.invertCanCoder.contains(module.canCoder),
                            0
                    );
        }

        // purposefully return null so that swerve modules default to quad encoders
        return canCoder;
    }

    public CANcoder getCanCoder(String subsystemName, String canCoderName, double offset) {
        var subsystem = getSubsystem(subsystemName);
        CANcoder canCoder = null;

        int id = subsystem.canCoders.get(canCoderName);
        if (subsystem.canCoders.containsKey(canCoderName) && id >= 0) {
            canCoder =
                    MotorFactory.createCanCoder(
                            id,
                            config.infrastructure.canBusName,
                            false, //Keeping this one false because "inverting" in phoenix 6 seriously messes it up,
                            offset
                    );
        }

        return canCoder;
    }

    @Nonnull
    public ISolenoid getSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented) {
            if (isHardwareValid(subsystem.solenoids, name) && isPcmEnabled()) {
                return new SolenoidImpl(
                        config.infrastructure.pcmId,
                        config.infrastructure.pcmIsRev
                                ? PneumaticsModuleType.REVPH
                                : PneumaticsModuleType.CTREPCM,
                        subsystem.solenoids.get(name)
                );
            }
            reportGhostWarning("Solenoid", subsystemName, name);
        }
        return new GhostSolenoid();
    }

    @Nonnull
    public IDoubleSolenoid getDoubleSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        if (getSubsystem(subsystemName).doubleSolenoids != null) {
            DoubleSolenoidConfig solenoidConfig = getSubsystem(subsystemName)
                    .doubleSolenoids.get(name);
            if (
                    subsystem.implemented &&
                            solenoidConfig != null &&
                            isHardwareValid(solenoidConfig.forward) &&
                            isHardwareValid(solenoidConfig.reverse) &&
                            isPcmEnabled()
            ) {
                return new DoubleSolenoidImpl(
                        config.infrastructure.pcmId,
                        PneumaticsModuleType.REVPH,
                        solenoidConfig.forward,
                        solenoidConfig.reverse
                );
            }
        }
        reportGhostWarning("DoubleSolenoid", subsystemName, name);
        return new GhostDoubleSolenoid();
    }

    public ILEDManager getLEDManager(String subsystemName) {
        var subsystem = getSubsystem(subsystemName);
        ILEDManager ledManager = null;
        if (subsystem.implemented) {
            if (isHardwareValid(subsystem.canifier)) {
                ledManager = new CanifierImpl(subsystem.canifier);
            } else if (isHardwareValid(subsystem.candle)) {
                ledManager =
                        new CANdleImpl(
                                subsystem.candle,
                                config.infrastructure.canBusName
                        );
            }
            if (ledManager != null) {
                ledManager.configFactoryDefault();
                ledManager.configStatusLedState(true);
                ledManager.configLOSBehavior(false);
                ledManager.configLEDType(CANdle.LEDStripType.BRG); // type config of the strip we use rn
                ledManager.configBrightnessScalar(1);
                return ledManager;
            }
            reportGhostWarning("LEDManager", subsystemName, "");
        }

        return new GhostLEDManager();
    }

    public ICompressor getCompressor() {
        if (isPcmEnabled() && config.infrastructure.compressorEnabled) {
            PneumaticsModuleType pcmType = config.infrastructure.pcmIsRev
                    ? PneumaticsModuleType.REVPH
                    : PneumaticsModuleType.CTREPCM;
            return new CompressorImpl(getPcmId(), pcmType);
        }
        reportGhostWarning("Compressor", "ROOT", "on PCM ID " + getPcmId()); // root?
        return new GhostCompressor();
    }

    private boolean isHardwareValid(Map<String, Integer> map, String name) {
        if (map != null) {
            Integer hardwareId = map.get(name);
            return hardwareId != null && hardwareId > -1 && RobotBase.isReal();
        }
        return false;
    }

    private boolean isHardwareValid(Integer hardwareId) {
        return hardwareId != null && hardwareId > -1 && RobotBase.isReal();
    }

    private boolean isMotorValid(Map<String, MotorConfiguration> map, String name) {
        if (map != null) {
            Integer hardwareId = map.get(name).id;
            return hardwareId != null && hardwareId > -1 && RobotBase.isReal();
        }
        return false;
    }

    public Map<String, Double> getConstants() {
        return config.constants;
    }

    public SubsystemConfig getSubsystem(String subsystemName) {
        if (config.subsystems.containsKey(subsystemName)) {
            var subsystem = config.subsystems.get(subsystemName);
            if (subsystem == null) {
                subsystem = new SubsystemConfig();
                subsystem.implemented = false;
                GreenLogger.log("Subsystem not defined: " + subsystemName);
            }
            return subsystem;
        }
        SubsystemConfig subsystem = new SubsystemConfig();
        subsystem.implemented = false;
        return subsystem;
    }

    public double getConstant(String name, double defaultVal) {
        if (getConstants() == null || !getConstants().containsKey(name)) {
            DriverStation.reportWarning("Yaml constants:" + name + " missing", true);
            return defaultVal;
        }
        return getConstants().get(name);
    }

    public String getInputHandlerName() {
        return Objects.requireNonNullElse(config.inputHandler, "empty");
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        return getConstant(subsystemName, name, defaultVal, true);
    }

    public double getConstant(
            String subsystemName,
            String name,
            double defaultVal,
            boolean showWarning
    ) {
        if (!getSubsystem(subsystemName).implemented) {
            return defaultVal;
        }
        if (
                getSubsystem(subsystemName).constants == null ||
                        !getSubsystem(subsystemName).constants.containsKey(name)
        ) {
            if (showWarning) {
                DriverStation.reportWarning(
                        "Yaml: subsystem \"" +
                                subsystemName +
                                "\" constant \"" +
                                name +
                                "\" missing",
                        defaultVal == 0
                );
            }
            return defaultVal;
        }
        return getSubsystem(subsystemName).constants.get(name);
    }

    public Slot0Configs getSwervePIDConfigs(String subsystemName, PIDConfig configType) {
        PIDSlotConfiguration configs = getPidSlotConfig(subsystemName, "slot0", configType);

        return new Slot0Configs()
                .withKP(configs.kP)
                .withKI(configs.kI)
                .withKD(configs.kD)
                .withKV(configs.kV)
                .withKS(configs.kS)
                .withKA(configs.kA)
                .withKG(configs.kG);
    }

    public PIDSlotConfiguration getPidSlotConfig(String subsystemName) {
        return getPidSlotConfig(subsystemName, "slot0", PIDConfig.Generic);
    }

    public PIDSlotConfiguration getPidSlotConfig(String subsystemName, String slot) {
        return getPidSlotConfig(subsystemName, slot, PIDConfig.Generic);
    }

    public PIDSlotConfiguration getPidSlotConfig(
            String subsystemName,
            String slot,
            PIDConfig configType
    ) {
        var subsystem = getSubsystem(subsystemName);
        Map<String, PIDSlotConfiguration> config = null;
        if (subsystem.implemented) {
            config = switch (configType) {
                case Azimuth -> subsystem.swerveModules.azimuthPID;
                case Drive -> subsystem.swerveModules.drivePID;
                case Generic -> subsystem.pidConfig;
            };
        }
        if (config != null && config.get(slot) != null) return config.get(slot);
        else {
            if (subsystem.implemented) {
                DriverStation.reportError(
                        "pidConfig missing for " + subsystemName + " " + slot,
                        true
                );
                return null;
            } else {
                // return a default config if not implemented
                return PIDUtil.createDefaultPIDSlotConfig();
            }
        }
    }

    public PowerDistribution getPd() {
        return new PowerDistribution(
                config.infrastructure.pdId,
                config.infrastructure.pdIsRev
                        ? PowerDistribution.ModuleType.kRev
                        : PowerDistribution.ModuleType.kCTRE
        );
    }

    public IPigeonIMU getPigeon() {
        int id = config.infrastructure.pigeonId;
        IPigeonIMU pigeon;
        if (!isHardwareValid(id)) {
            pigeon = new GhostPigeonIMU(id);
        } else if (config.infrastructure.isPigeon2) {
            GreenLogger.log("Using Pigeon 2 for id: " + id);
            pigeon = new Pigeon2Impl(id, config.infrastructure.canBusName);
        } else {
            GreenLogger.log("Using old Pigeon for id: " + id);
            pigeon = new PigeonIMUImpl(id);
        }
        if (getConstant("resetFactoryDefaults", 0) > 0) {
            pigeon.configFactoryDefaults();
            pigeon.set_StatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 100);
            pigeon.set_StatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 100);
        }
        return pigeon;
    }

    public IProximitySensor getProximitySensor(String name) {
        if (config.infrastructure.proximitySensors == null) {
            return new GhostProximitySensor();
        }
        int id = config.infrastructure.proximitySensors.getOrDefault(name, -1);
        if (id < 0) {
            GreenLogger.log("Incorrect Name: Proximity sensor not found, using ghost!");
            return new GhostProximitySensor();
        }
        GreenLogger.log("Creating Proximity Sensor: " + name + " at port: " + id);
        return new ProximitySensor(name, config.infrastructure.proximitySensors.get(name));
    }

    public int getPcmId() {
        if (config.infrastructure == null && config.infrastructure.pcmId == null) return -1;
        return config.infrastructure.pcmId;
    }

    public boolean isPcmEnabled() {
        return getPcmId() > -1;
    }

    public boolean isCompressorEnabled() {
        return config.infrastructure.compressorEnabled;
    }

    public String getCanBusName() {
        return config.infrastructure.canBusName;
    }

    public int getPigeonID() {
        return config.infrastructure.pigeonId;
    }

    private void reportGhostWarning(
            String type,
            String subsystemName,
            String componentName
    ) {
        GreenLogger.log(
                "  " +
                        type +
                        " \"" +
                        componentName +
                        "\" invalid in Yaml for subsystem \"" +
                        subsystemName +
                        "\", using ghost!"
        );
    }

    private enum PIDConfig {
        Azimuth,
        Drive,
        Generic,
    }
}