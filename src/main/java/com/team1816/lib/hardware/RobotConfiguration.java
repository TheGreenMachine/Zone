
package com.team1816.lib.hardware;

import java.util.Map;

public class RobotConfiguration {

    public Map<String, SubsystemConfig> subsystems;
    public InfrastructureConfig infrastructure;
    public Map<String, Double> constants;
    public java.lang.String inputHandler;

    @Override
    public java.lang.String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(RobotConfiguration.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("subsystems");
        sb.append('=');
        sb.append(((this.subsystems == null)?"<null>":this.subsystems));
        sb.append(',');
        sb.append("infrastructure");
        sb.append('=');
        sb.append(((this.infrastructure == null)?"<null>":this.infrastructure));
        sb.append(',');
        sb.append("constants");
        sb.append('=');
        sb.append(((this.constants == null)?"<null>":this.constants));
        sb.append(',');
        sb.append("inputHandler");
        sb.append('=');
        sb.append(((this.inputHandler == null)?"<null>":this.inputHandler));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
