package org.usfirst.frc.team1318.robot.Common.ToggleButtons;

/**
 * Defines a toggle that cycles through multiple different states
 * 
 * @author Will
 *
 */
public class MultiToggleButton implements IMultiToggle
{
    private int currentStatePosition;
    private int[] possibleStates;

    /**
     * Initializes a new MultiToggleButton
     * @param possibleStates for the toggle
     */
    public MultiToggleButton(int[] possibleStates)
    {
        this.possibleStates = possibleStates;
        this.currentStatePosition = 0;
    }

    /**
     * Attempt to change the current state
     */
    public void toggle()
    {
        this.currentStatePosition++;
        
        // if we are going past the end of the list, go back to the beginning...
        if (this.currentStatePosition >= this.possibleStates.length)
        {
            this.currentStatePosition = 0;
        }
    }

    /**
     * Gets a value indicating the currently toggled state
     * @return current toggle state
     */
    public int getToggledState()
    {
        return this.possibleStates[this.currentStatePosition];
    }
}
