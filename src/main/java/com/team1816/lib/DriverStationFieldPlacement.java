package com.team1816.lib;

import com.team1816.lib.auto.FieldPlacement;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Map;
import java.util.Optional;

public enum DriverStationFieldPlacement {;
    private static Map<FieldPlacement, Optional<DriverStationFieldPlacement.AutoPlacement>> m_FieldPlacementMap;

    public static enum AutoPlacement{
        Top,
        Middle,
        Bottom;

        private AutoPlacement() {
        }
    }
    public static Optional<DriverStationFieldPlacement.AutoPlacement> getAutoPlacement() {
        FieldPlacement fieldPlacement = FieldPlacementJNI.getAutoPlacement();
        if (fieldPlacement == null) {
            fieldPlacement = FieldPlacement.UNKNOWN;
        }

        return (Optional)m_FieldPlacementMap.get(fieldPlacement);
    }
}
