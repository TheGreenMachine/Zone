
package com.team1816.lib.input_handler;

import java.util.Map;

public class InputHandlerConfig {
    public ControllerConfig driver;
    public ControllerConfig operator;
    public Map<String, String> buttonboard;

    @Override
    public java.lang.String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(InputHandlerConfig.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("driver");
        sb.append('=');
        sb.append(((this.driver == null)?"<null>":this.driver));
        sb.append(',');
        sb.append("operator");
        sb.append('=');
        sb.append(((this.operator == null)?"<null>":this.operator));
        sb.append(',');
        sb.append("buttonboard");
        sb.append('=');
        sb.append(((this.buttonboard == null)?"<null>":this.buttonboard));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
