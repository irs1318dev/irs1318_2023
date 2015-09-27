package org.usfirst.frc.team1318.robot.Driver.Buttons;

public class SimpleButton implements IButton
{
    private final boolean activateOnPress;

    private boolean activated;

    /**
     * Initializes a new SimpleButton that will be considered activate when pressed
     */
    public SimpleButton()
    {
        this(true);
    }

    /**
     * Initializes a new SimpleButton
     * @param activateOnPress indicates whether to consider the button activated when the key is pressed or when released 
     */
    public SimpleButton(boolean activateOnPress)
    {
        this.activateOnPress = activateOnPress;
    }

    /**
     * attempt to activate button, otherwise sets activated to false 
     * @param buttonState the current position of the button (whether it is currently pressed) 
     */
    public void updateState(boolean buttonState)
    {
        if ((this.activateOnPress && buttonState)
            || (!this.activateOnPress && !buttonState))
        {
            this.activated = true;
        }
        else
        {
            this.activated = false;
        }
    }

    /**
     * gets a value indicating whether the button is activated 
     * @return true for act, otherwise false 
     */
    public boolean isActivated()
    {
        return this.activated;
    }
}
