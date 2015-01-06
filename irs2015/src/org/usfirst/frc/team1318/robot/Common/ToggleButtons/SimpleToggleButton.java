package org.usfirst.frc.team1318.robot.Common.ToggleButtons;

/**
 * Defines a simple toggle that switches between true and false.
 * 
 * @author Will
 *
 */
public class SimpleToggleButton implements ISimpleToggle
{
    private boolean currentState;
    private boolean buttonState;

    /**
     * Initializes a new SimpleToggleButton
     */
    public SimpleToggleButton()
    {
        this.currentState = false;
        this.buttonState = false;
    }

    /**
     * change current state if it needs to be changed based on button values 
     * @param newState current button state 
     */
    public void updateState(boolean newState)
    {
    	if(buttonState && !newState)
    	{
    		currentState = !currentState;
    	}
    	
    	buttonState = newState;
    }
    
    /**
     * Attempt to change the current state
     */
    public void toggle()
    {
        this.currentState = !this.currentState;
    }

    /**
     * Gets a value indicating whether this is currently toggled
     * @return true if toggled, otherwise false
     */
    public boolean isToggled()
    {
        return this.currentState;
    }
}
