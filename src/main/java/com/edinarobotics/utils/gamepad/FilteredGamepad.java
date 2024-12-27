package com.edinarobotics.utils.gamepad;

import com.edinarobotics.utils.gamepad.gamepadfilters.GamepadFilterSet;
import com.edinarobotics.utils.math.Vector2;

/**
 * Implements a Gamepad that filters all of its joystick axis values through
 * a given GamepadFilterSet.
 */
public class FilteredGamepad extends Gamepad {
    private GamepadFilterSet filters;
    private Vector2 left;
    private Vector2 right;
    private GamepadAxisState state;
    
    /**
     * Constructs a new FilteredGamepad that will send the axis results
     * of the gamepad on the given port through the given GamepadFilterSet.
     * @param port The port of the gamepad that is to be wrapped by this
     * FilteredGamepad.
     * @param filterSet The GamepadFilterSet through which all joystick
     * values are to be sent.
     */
    public FilteredGamepad(int port, GamepadFilterSet filterSet){
        super(port);
        this.filters = filterSet;
        this.left = new Vector2(0, 0);
        this.right = new Vector2(0, 0);
        this.state = new GamepadAxisState(left, right);
    }
    
    /**
     * Returns the state of the left joystick as a Vector2.
     * This vector 2 contains the state of the x- and y- axis of the joystick.
     * @return A Vector2 representing the state of the left joystick after
     * being filtered by the given GamepadFilterSet.
     */
    public Vector2 getLeftJoystick(){
        return getGamepadAxisState().getLeftJoystick();
    }
    
    /**
     * Returns the state of the right joystick as a Vector2.
     * This vector 2 contains the state of the x- and y- axis of the joystick.
     * @return A Vector2 representing the state of the right joystick after
     * being filtered by the given GamepadFilterSet.
     */
    public Vector2 getRightJoystick(){
        return getGamepadAxisState().getRightJoystick();
    }
    
    /**
     * Returns the state of the gamepad's joysticks together in a
     * GamepadAxisState. The values in this object have been filtered
     * by the given GamepadFilterSet.
     * @return A GamepadAxisState object containing the states of all the
     * joystick axes on this Gamepad.
     */
    @Override
    public GamepadAxisState getGamepadAxisState() {
        // This method recomputes values so we avoid infinite loops
        // in FilteredGamepad.
        double leftX = super.getLeftX();
        double leftY = super.getLeftY();
        double rightX = super.getRightX();
        double rightY = super.getRightY();
        left.set(leftX, leftY);
        right.set(rightX, rightY);
        return filters.filter(state);
    }
    
    /**
     * Returns the current value of the x-axis of the left joystick. <br/>
     * A value of {@code -1} indicates that the joystick is fully left.<br/>
     * A value of {@code 1} indicates that the joystick is fully right.
     * @return The current value of the x-axis of the left joystick after
     * being sent through the given GamepadFilterSet.
     */
    public double getLeftX(){
        return getLeftJoystick().getX();
    }
    
    /**
     * Returns the current value of the y-axis of the left joystick. <br/>
     * A value of {@code -1} indicates that the joystick is fully down.<br/>
     * A value of {@code 1} indicates that the joystick is fully up.
     * @return The current value of the y-axis of the left joystick after
     * being sent through the given GamepadFilterSet.
     */
    public double getLeftY(){
        return getLeftJoystick().getY();
    }
    
    /**
     * Returns the current value of the x-axis of the right joystick. <br/>
     * A value of {@code -1} indicates that the joystick is fully left.<br/>
     * A value of {@code 1} indicates that the joystick is fully right.
     * @return The current value of the x-axis of the right joystick after
     * being sent through the given GamepadFilterSet.
     */
    public double getRightX(){
        return getRightJoystick().getX();
    }
    
    /**
     * Returns the current value of the y-axis of the right joystick. <br/>
     * A value of {@code -1} indicates that the joystick is fully down.<br/>
     * A value of {@code 1} indicates that the joystick is fully up.
     * @return The current value of the y-axis of the right joystick after
     * being sent through the given GamepadFilterSet.
     */
    public double getRightY(){
        return getRightJoystick().getY();
    }
}
