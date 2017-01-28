package org.usfirst.frc.team1318.robot.general;

import org.usfirst.frc.team1318.robot.common.ComplementaryFilter;
import org.usfirst.frc.team1318.robot.common.IController;
import org.usfirst.frc.team1318.robot.driver.Driver;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * Power manager.
 * 
 * @author Will
 *
 */
public class PowerManager implements IController
{
    private final PowerDistributionPanel pdp;
    private ComplementaryFilter filter;

    /**
     * Initializes a new PowerComponent
     */
    public PowerManager()
    {
        this.pdp = new PowerDistributionPanel();
        this.filter = new ComplementaryFilter(0.4, 0.6, this.pdp.getVoltage());
    }

    public double getVoltage()
    {
        return this.filter.getValue();
    }

    @Override
    public void update()
    {
        this.filter.update(this.pdp.getVoltage());
    }

    @Override
    public void stop()
    {
    }

    @Override
    public void setDriver(Driver driver)
    {
        // no-op
    }
}
