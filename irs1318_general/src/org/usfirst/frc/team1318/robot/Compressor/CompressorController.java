package org.usfirst.frc.team1318.robot.Compressor;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Driver.Driver;

/**
 * Compressor controller.
 * The controller defines the logic that controls a mechanism given inputs (component) and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs (component).
 * 
 * @author Will
 *
 */
public class CompressorController implements IController
{
    private boolean isStarted;
    private final CompressorComponent component;

    /**
     * Initializes a new CompressorController
     * @param component to control
     */
    public CompressorController(CompressorComponent component)
    {
        this.component = component;
        this.isStarted = false;
    }

    /**
     * set the driver that the controller should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        // not needed for this controller
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant component
     */
    @Override
    public void update()
    {
        if (!this.isStarted)
        {
            this.component.start();
            this.isStarted = true;
        }
    }

    /**
     * stop the relevant component
     */
    @Override
    public void stop()
    {
        this.component.stop();
        this.isStarted = false;
    }
}
