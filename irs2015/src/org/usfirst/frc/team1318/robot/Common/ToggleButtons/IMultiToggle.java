package org.usfirst.frc.team1318.robot.Common.ToggleButtons;

/**
 * Describes a toggle that cycles through multiple different states
 * 
 * @author Will
 * 
 */
public interface IMultiToggle<T> extends IToggle
{
    /**
     * Gets a value indicating the currently toggled state
     * @return current toggle state
     */
    public T getToggledState();
}
