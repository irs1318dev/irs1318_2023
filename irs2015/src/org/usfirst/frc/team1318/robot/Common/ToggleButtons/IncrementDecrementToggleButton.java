package org.usfirst.frc.team1318.robot.Common.ToggleButtons;

/**
 * Defines a toggle that allows one to increment or decrement through multiple different states
 * 
 *       decrement    start    increment
 *          <--         |         -->
 *                     \|/
 * -5, -4, -3, -2, -1,  0,  1,  2,  3,  4,  5
 * 
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
public class IncrementDecrementToggleButton<T> implements IMultiToggle<T>, IIncrementDecrementToggle
{
    private final boolean toggleOnPress;

    private final int startPosition;

    private int currentStatePosition;
    private T[] possibleStates;
    private boolean prevIncrementButtonState;
    private boolean prevDecrementButtonState;

    /**
     * Initializes a new IncrementDecrementToggleButton
     * @param possibleStates for the toggle
     */
    public IncrementDecrementToggleButton(T[] possibleStates)
    {
        this(possibleStates, 0, true);
    }

    /**
     * Initializes a new IncrementDecrementToggleButton
     * @param startPosition index within the possible states array where we should start
     * @param possibleStates for the toggle
     */
    public IncrementDecrementToggleButton(T[] possibleStates, int startPosition)
    {
        this(possibleStates, startPosition, true);
    }

    /**
     * Initializes a new IncrementDecrementToggleButton
     * @param possibleStates for the toggle
     * @param startPosition index within the possible states array where we should start
     * @param toggleOnPress indicates whether we should toggle when the button is first pressed or when released
     */
    public IncrementDecrementToggleButton(T[] possibleStates, int startPosition, boolean toggleOnPress)
    {
        this.possibleStates = possibleStates;
        this.currentStatePosition = startPosition;
        this.prevIncrementButtonState = false;
        this.prevDecrementButtonState = false;

        this.toggleOnPress = toggleOnPress;
        this.startPosition = startPosition;
    }

    /**
     * Attempt to change the current state - note that if both buttons are pressed, they will compete
     * @param incrementButtonState the current position of the increment button (whether it is currently pressed)
     * @param decrementButtonState the current position of the decrement button (whether it is currently pressed)
     */
    public void updateState(boolean incrementButtonState, boolean decrementButtonState)
    {
        // if increment button has switched state, check if we want to toggle
        if (this.prevIncrementButtonState != incrementButtonState &&
            (this.toggleOnPress && incrementButtonState || !this.toggleOnPress && !incrementButtonState))
        {
            // only go up to position possibleStates.length - 1
            if (this.currentStatePosition < this.possibleStates.length - 1)
            {
                this.currentStatePosition++;
            }
        }

        this.prevIncrementButtonState = incrementButtonState;

        // if decrement button has switched state, check if we want to toggle
        if (this.prevDecrementButtonState != decrementButtonState &&
            (this.toggleOnPress && decrementButtonState || !this.toggleOnPress && !decrementButtonState))
        {
            // only go down to position 0
            if (this.currentStatePosition > 0)
            {
                this.currentStatePosition--;
            }
        }

        this.prevDecrementButtonState = decrementButtonState;
    }

    /**
     * Gets a value indicating the currently toggled state
     * @return current toggle state
     */
    public T getToggledState()
    {
        return this.possibleStates[this.currentStatePosition];
    }

    /**
     * Reset the state of the toggle button to the starting state
     */
    public void reset()
    {
        this.currentStatePosition = this.startPosition;
    }
}
