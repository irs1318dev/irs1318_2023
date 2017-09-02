package org.usfirst.frc.team1318.robot.compressor;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.wpilib.ICompressor;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.driver.common.Driver;

import com.google.inject.Inject;

/**
 * Compressor mechanism.
 * The mechanism defines the logic that controls a mechanism given inputs and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs.
 * 
 * @author Will
 *
 */
@Singleton
public class CompressorMechanism implements IMechanism
{
    private final ICompressor compressor;

    private boolean isStarted;

    /**
     * Initializes a new CompressorMechanism
     * @param provider for obtaining electronics objects
     */
    @Inject
    public CompressorMechanism(IWpilibProvider provider)
    {
        this.compressor = provider.getCompressor();
        this.isStarted = false;
    }

    /**
     * set the driver that the mechanism should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        // not needed for this controller
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        if (!this.isStarted)
        {
            this.compressor.start();
            this.isStarted = true;
        }
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.compressor.stop();
        this.isStarted = false;
    }
}
