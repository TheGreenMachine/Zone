
package com.team1816.lib.hardware;

import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

public class SubsystemConfig {

    public Boolean implemented;
    public Object required;
   public Map<String, MotorConfiguration> motors;
    public java.lang.Integer canifier;
    public java.lang.Integer candle;
    public Map<String, Double> constants;
    public Map<String, Integer> canCoders;
    public SensorMapConfiguration sensors;
    public SwerveModuleConfiguration swerveModules;
    public Map<String, Integer> solenoids;
    public Map<String, DoubleSolenoidConfig> doubleSolenoids;
    public Set<java.lang.String> invertMotor = new LinkedHashSet<java.lang.String>();
    public Set<java.lang.String> invertSensorPhase = new LinkedHashSet<java.lang.String>();
    public Set<java.lang.String> invertCanCoder = new LinkedHashSet<java.lang.String>();
    public Map<String, PIDSlotConfiguration> pidConfig;

    @Override
    public java.lang.String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(SubsystemConfig.class.getName()).append('@').append(java.lang.Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("implemented");
        sb.append('=');
        sb.append(((this.implemented == null)?"<null>":this.implemented));
        sb.append(',');
        sb.append("required");
        sb.append('=');
        sb.append(((this.required == null)?"<null>":this.required));
        sb.append(',');
        sb.append("motors");
        sb.append('=');
        sb.append(((this.motors == null)?"<null>":this.motors));
        sb.append(',');
        sb.append("canifier");
        sb.append('=');
        sb.append(((this.canifier == null)?"<null>":this.canifier));
        sb.append(',');
        sb.append("candle");
        sb.append('=');
        sb.append(((this.candle == null)?"<null>":this.candle));
        sb.append(',');
        sb.append("constants");
        sb.append('=');
        sb.append(((this.constants == null)?"<null>":this.constants));
        sb.append(',');
        sb.append("canCoders");
        sb.append('=');
        sb.append(((this.canCoders == null)?"<null>":this.canCoders));
        sb.append(',');
        sb.append("sensors");
        sb.append('=');
        sb.append(((this.sensors == null)?"<null>":this.sensors));
        sb.append(',');
        sb.append("swerveModules");
        sb.append('=');
        sb.append(((this.swerveModules == null)?"<null>":this.swerveModules));
        sb.append(',');
        sb.append("solenoids");
        sb.append('=');
        sb.append(((this.solenoids == null)?"<null>":this.solenoids));
        sb.append(',');
        sb.append("doubleSolenoids");
        sb.append('=');
        sb.append(((this.doubleSolenoids == null)?"<null>":this.doubleSolenoids));
        sb.append(',');
        sb.append("invertMotor");
        sb.append('=');
        sb.append(((this.invertMotor == null)?"<null>":this.invertMotor));
        sb.append(',');
        sb.append("invertSensorPhase");
        sb.append('=');
        sb.append(((this.invertSensorPhase == null)?"<null>":this.invertSensorPhase));
        sb.append(',');
        sb.append("invertCanCoder");
        sb.append('=');
        sb.append(((this.invertCanCoder == null)?"<null>":this.invertCanCoder));
        sb.append(',');
        sb.append("pidConfig");
        sb.append('=');
        sb.append(((this.pidConfig == null)?"<null>":this.pidConfig));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
