package org.usfirst.frc.team1318.robot.Common.ToggleButtons;

/**
 * Describes a toggle that gets un-toggled automatically after a certain period of time elapses.
 * 
 * @author Will
 *
 */
public interface ITimedToggle
{
    /**
     * Gets a value indicating whether we can toggle
     * @return true if we can toggle, otherwise false
     */
    public boolean canToggle();

    /**
     * Indicates that some time has passed
     */
    public void update();
}
