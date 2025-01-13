package com.team1816.lib.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModuleRequest implements LegacySwerveRequest {
    public SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    public ModuleRequest withModuleStates(SwerveModuleState... moduleStates) {
        this.moduleStates = moduleStates;
        return this;
    }

    @Override
    public StatusCode apply(LegacySwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        for (int i = 0; i < 4; i++) {
            modulesToApply[i].apply(moduleStates[i], LegacySwerveModule.DriveRequestType.Velocity, LegacySwerveModule.SteerRequestType.MotionMagic);
        }
        return StatusCode.OK;
    }

    @Override
    public StatusCode apply(LegacySwerveControlRequestParameters legacySwerveControlRequestParameters, LegacySwerveModule... legacySwerveModules) {
        return null;
    }
}
