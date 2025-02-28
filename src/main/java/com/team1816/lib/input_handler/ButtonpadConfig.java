
package com.team1816.lib.input_handler;

public class ButtonpadConfig {

    public String x;
    public String y;
    public String a;
    public String b;

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(ButtonpadConfig.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("x");
        sb.append('=');
        sb.append(((this.x == null)?"<null>":this.x));
        sb.append(',');
        sb.append("y");
        sb.append('=');
        sb.append(((this.y == null)?"<null>":this.y));
        sb.append(',');
        sb.append("a");
        sb.append('=');
        sb.append(((this.a == null)?"<null>":this.a));
        sb.append(',');
        sb.append("b");
        sb.append('=');
        sb.append(((this.b == null)?"<null>":this.b));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
