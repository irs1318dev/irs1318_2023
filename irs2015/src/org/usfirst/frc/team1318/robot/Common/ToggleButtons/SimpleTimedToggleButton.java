package org.usfirst.frc.team1318.robot.Common.ToggleButtons;

import edu.wpi.first.wpilibj.Timer;

/**
 * Defines a simple true/false toggle that gets un-toggled automatically after a certain period of time elapses.
 * 
 * @author Will
 *
 */
public class SimpleTimedToggleButton extends SimpleToggleButton implements ITimedToggle
{
    private final double toggleDuration;

    private Timer timer;
    private Double startTime;

    /**
     * Initializes a new SimpleTimedToggleButton
     * @param toggleDuration time duration for the toggle
     */
    public SimpleTimedToggleButton(double toggleDuration)
    {
    	super(true);

        this.toggleDuration = toggleDuration;
        this.timer = new Timer();
        this.timer.start();
        this.startTime = null;
    }

    /**
     * Gets a value indicating whether we can toggle
     * @return true if we can toggle, otherwise false
     */
    public boolean canToggle()
    {
        return this.startTime == null;
    }

    /**
     * Indicates that some time has passed
     */
    public void update()
    {
        if (this.startTime != null &&
            this.timer.get() > this.startTime + this.toggleDuration)
        {
            this.startTime = null;
        }
    }

    /**
     * Attempt to change the current state
     * @param buttonState the current position of the button (whether it is currently pressed)
     */
    public void updateState(boolean buttonState)
    {
		super.updateState(buttonState && this.canToggle());
		
		if (this.canToggle())
		{
			this.startTime = this.timer.get();
		}
    }

    /**
     * Cancel the toggle timing
     */
    public void cancel()
    {
        this.startTime = null;
    }
}
