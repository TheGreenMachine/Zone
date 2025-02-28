
package com.team1816.lib.input_handler;

public class DpadConfig {
    public String up;
    public String down;
    public String left;
    public String right;

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(DpadConfig.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("up");
        sb.append('=');
        sb.append(((this.up == null)?"<null>":this.up));
        sb.append(',');
        sb.append("down");
        sb.append('=');
        sb.append(((this.down == null)?"<null>":this.down));
        sb.append(',');
        sb.append("left");
        sb.append('=');
        sb.append(((this.left == null)?"<null>":this.left));
        sb.append(',');
        sb.append("right");
        sb.append('=');
        sb.append(((this.right == null)?"<null>":this.right));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
