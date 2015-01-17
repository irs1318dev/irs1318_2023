package org.usfirst.frc.team1318.robot.Common.ToggleButtons;

/**
 * Defines a toggle that cycles through multiple different states
 * 
 * Toggle on press behavior:
 * 
 *     button pressed:        _________________
 *                           |                 |
 * button not pressed: ______|                 |________
 *                           ^ takes effect when first pressed
 * 
 * Toggle on release behavior:
 * 
 *     button pressed:        _________________
 *                           |                 |
 * button not pressed: ______|                 |________
 *                                             ^ takes effect when first released
 * 
 * 
 * @author Will
 *
 */
public class MultiToggleButton<T> implements IMultiToggle<T>, IToggle
{
    private final boolean toggleOnPress;

    private int currentStatePosition;
    private T[] possibleStates;
    private boolean prevButtonState;

    /**
     * Initializes a new MultiToggleButton
     * @param possibleStates for the toggle
     */
    public MultiToggleButton(T[] possibleStates)
    {
        this(possibleStates, true);
    }

    /**
     * Initializes a new MultiToggleButton
     * @param possibleStates for the toggle
     * @param toggleOnPress indicates whether we should toggle when the button is first pressed or when released
     */
    public MultiToggleButton(T[] possibleStates, boolean toggleOnPress)
    {
        this.possibleStates = possibleStates;
        this.currentStatePosition = 0;
        this.prevButtonState = false;

        this.toggleOnPress = toggleOnPress;
    }

    /**
     * Attempt to change the current state
     * @param buttonState the current position of the button (whether it is currently pressed)
     */
    public void updateState(boolean buttonState)
    {
        // if button has switched state, check if we want to toggle
        if (this.prevButtonState != buttonState &&
            (this.toggleOnPress && buttonState || !this.toggleOnPress && !buttonState))
        {
            this.currentStatePosition++;

            // if we are going past the end of the list, go back to the beginning...
            if (this.currentStatePosition >= this.possibleStates.length)
            {
                this.currentStatePosition = 0;
            }
        }

        this.prevButtonState = buttonState;
    }

    /**
     * Gets a value indicating the currently toggled state
     * @return current toggle state
     */
    public T getToggledState()
    {
        return this.possibleStates[this.currentStatePosition];
    }
}
