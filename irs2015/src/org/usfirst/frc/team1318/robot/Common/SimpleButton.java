package org.usfirst.frc.team1318.robot.Common;

public class SimpleButton
{

    private boolean activateOnPress;

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
        prevState = false;
        this.activateOnPress = activateOnPress;
    }

    /**
     * attempt to activate 
     * @param buttonState the current position of the button (whether it is currently pressed) 
     */
    public void updateState(boolean buttonState)
    {
        if (activateOnPress && !prevState && buttonState)
        {
            activated = true;
        }
        else if (!activateOnPress && prevState && !buttonState)
        {
            activated = true;
        }
        prevState = buttonState;
    }

    /**
     * gets a value indicating whether the button is activated 
     * @return true for act, otherwise false 
     */
    public boolean isActivated()
    {
        if (activated)
        {
            activated = false;
            return true;
        }
        return false;
    }

}
