package org.usfirst.frc.team1318.robot.general;

import org.usfirst.frc.team1318.robot.common.ComplementaryFilter;
import org.usfirst.frc.team1318.robot.common.IController;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IPowerDistributionPanel;
import org.usfirst.frc.team1318.robot.driver.Driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Power manager.
 * 
 * @author Will
 *
 */
@Singleton
public class PowerManager implements IController
{
    private final IPowerDistributionPanel pdp;
    private ComplementaryFilter batteryVoltageFilter;

    /**
     * Initializes a new PowerComponent
     */
    @Inject
    public PowerManager(IPowerDistributionPanel pdp)
    {
        this.pdp = pdp;
        this.batteryVoltageFilter = new ComplementaryFilter(0.4, 0.6, this.pdp.getBatteryVoltage());
    }

    public double getCurrent(int pdpChannel)
    {
        return this.pdp.getCurrent(pdpChannel);
    }

    public double getBatteryVoltage()
    {
        return this.batteryVoltageFilter.getValue();
    }

    @Override
    public void update()
    {
        this.batteryVoltageFilter.update(this.pdp.getBatteryVoltage());
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
