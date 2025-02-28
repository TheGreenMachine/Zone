
package com.team1816.lib.input_handler;

public class JoystickConfig {
    public String horizontal;
    public String vertical;

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(JoystickConfig.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("horizontal");
        sb.append('=');
        sb.append(((this.horizontal == null)?"<null>":this.horizontal));
        sb.append(',');
        sb.append("vertical");
        sb.append('=');
        sb.append(((this.vertical == null)?"<null>":this.vertical));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
