package com.team1816.lib.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.sql.SQLOutput;

public class ModuleRequest implements LegacySwerveRequest {
    public SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    public ModuleRequest withModuleStates(SwerveModuleState... moduleStates) {
        this.moduleStates = moduleStates;
        return this;
    }

    @Override//LegacySwerveModule may not be the right class: if there are no issues, delete comment
    public StatusCode apply(LegacySwerveControlRequestParameters legacySwerveControlRequestParameters, LegacySwerveModule... modulesToApply) {
        for (int i = 0; i < 4; i++) {
            modulesToApply[i].apply(moduleStates[i], LegacySwerveModule.DriveRequestType.Velocity, LegacySwerveModule.SteerRequestType.MotionMagic);
        }
        return StatusCode.OK;
    }
}
