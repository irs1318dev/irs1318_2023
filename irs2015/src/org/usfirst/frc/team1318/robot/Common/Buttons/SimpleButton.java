package org.usfirst.frc.team1318.robot.Common.Buttons;

public class SimpleButton
{
    private final boolean activateOnPress;

    private boolean prevState;
    private boolean activated;

    /**
     * Initializes a new SimpleButton that will activate when pressed 
     */
    public SimpleButton()
    {
        this(true);
    }

    /**
     * Initializes a new SimpleButton 
     * @param activateOnPress indicates whether to activate when pressed or when released 
     */
    public SimpleButton(boolean activateOnPress)
    {
        this.prevState = false;
        this.activateOnPress = activateOnPress;
    }

    /**
     * attempt to activate button, otherwise sets activated to false 
     * @param buttonState the current position of the button (whether it is currently pressed) 
     */
    public void updateState(boolean buttonState)
    {
        if (this.activateOnPress && !this.prevState && buttonState)
        {
            this.activated = true;
        }
        else if (!this.activateOnPress && this.prevState && !buttonState)
        {
            this.activated = true;
        }
        else
        {
            this.activated = false;
        }

        this.prevState = buttonState;
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
