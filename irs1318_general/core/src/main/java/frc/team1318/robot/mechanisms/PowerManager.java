package frc.team1318.robot.mechanisms;

import frc.team1318.robot.common.ComplementaryFilter;
import frc.team1318.robot.common.IMechanism;
import frc.team1318.robot.common.robotprovider.IPowerDistributionPanel;
import frc.team1318.robot.common.robotprovider.IRobotProvider;
import frc.team1318.robot.driver.common.Driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Power manager.
 * 
 * @author Will
 *
 */
@Singleton
public class PowerManager implements IMechanism
{
    private final IPowerDistributionPanel pdp;

    private ComplementaryFilter batteryVoltageFilter;

    /**
     * Initializes a new PowerManager
     * @param provider for obtaining electronics objects
     */
    @Inject
    public PowerManager(IRobotProvider provider)
    {
        this.pdp = provider.getPDP();
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
    public void readSensors()
    {
        this.batteryVoltageFilter.update(this.pdp.getBatteryVoltage());
    }

    @Override
    public void update()
    {
        // no outputs to update for this mechanism
    }

    @Override
    public void stop()
    {
        this.batteryVoltageFilter.reset();
    }

    @Override
    public void setDriver(Driver driver)
    {
        // no-op
    }
}
