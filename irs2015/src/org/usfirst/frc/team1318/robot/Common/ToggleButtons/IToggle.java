package org.usfirst.frc.team1318.robot.Common.ToggleButtons;

/**
 * Describes something that toggles between 2 or more states.
 * 
 * @author Will
 *
 */
public interface IToggle
{
    /**
     * Attempt to change the current state
     * @param buttonState the current position of the button (whether it is currently pressed)
     */
    public void updateState(boolean buttonState);
}
