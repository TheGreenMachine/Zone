
package com.team1816.lib.input_handler;

import java.util.Map;

public class ControllerConfig {

    public java.lang.String controllerType;
    public Boolean rumble;
    public Map<String, JoystickConfig> joysticks;
    public Map<String, String> axes;
    public DpadConfig dpad;
    public ButtonpadConfig buttonpad;
    public Map<String, String> buttons;

    @Override
    public java.lang.String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(ControllerConfig.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("controllerType");
        sb.append('=');
        sb.append(((this.controllerType == null)?"<null>":this.controllerType));
        sb.append(',');
        sb.append("rumble");
        sb.append('=');
        sb.append(((this.rumble == null)?"<null>":this.rumble));
        sb.append(',');
        sb.append("joysticks");
        sb.append('=');
        sb.append(((this.joysticks == null)?"<null>":this.joysticks));
        sb.append(',');
        sb.append("axes");
        sb.append('=');
        sb.append(((this.axes == null)?"<null>":this.axes));
        sb.append(',');
        sb.append("dpad");
        sb.append('=');
        sb.append(((this.dpad == null)?"<null>":this.dpad));
        sb.append(',');
        sb.append("buttonpad");
        sb.append('=');
        sb.append(((this.buttonpad == null)?"<null>":this.buttonpad));
        sb.append(',');
        sb.append("buttons");
        sb.append('=');
        sb.append(((this.buttons == null)?"<null>":this.buttons));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
