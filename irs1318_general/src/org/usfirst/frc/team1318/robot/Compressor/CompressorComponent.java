package org.usfirst.frc.team1318.robot.Compressor;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;

import edu.wpi.first.wpilibj.Compressor;

/**
 * The compressor component class describes the electronics of the compressor and defines the abstract way to control it.
 * The electronics include just a compressor. 
 *  
 * @author Will
 *
 */
public class CompressorComponent
{
    private final Compressor compressor;

    /**
     * Initializes a new CompressorComponent
     */
    public CompressorComponent()
    {
        this.compressor = new Compressor(ElectronicsConstants.PCM_B_MODULE);
    }

    /**
     * Start the compressor 
     */
    public void start()
    {
        this.compressor.start();
    }

    /**
     * Stop the compressor
     */
    public void stop()
    {
        this.compressor.stop();
    }
}
