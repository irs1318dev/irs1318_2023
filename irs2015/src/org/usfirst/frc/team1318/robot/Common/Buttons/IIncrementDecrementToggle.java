package org.usfirst.frc.team1318.robot.Common.Buttons;

/**
 * Describes a toggle that cycles up and down through multiple different states
 * 
 * @author Will
 * 
 */
public interface IIncrementDecrementToggle
{
    /**
     * Attempt to change the current state
     * @param incrementButtonState the current position of the increment button (whether it is currently pressed)
     * @param decrementButtonState the current position of the decrement button (whether it is currently pressed)
     */
    public void updateState(boolean incrementButtonState, boolean decrementButtonState);

    /**
     * Reset the state of the toggle button to the starting state
     */
    public void reset();
}
