package org.usfirst.frc.team1318.robot.Driver.Buttons;

public interface IButton
{
    /**
     * attempt to activate button, otherwise sets activated to false 
     * @param buttonState the current position of the button (whether it is currently pressed) 
     */
    public void updateState(boolean buttonState);

    /**
     * gets a value indicating whether the button is activated 
     * @return true for act, otherwise false 
     */
    public boolean isActivated();
}
