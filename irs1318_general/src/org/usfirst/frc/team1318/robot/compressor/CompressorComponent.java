package org.usfirst.frc.team1318.robot.compressor;

import org.usfirst.frc.team1318.robot.common.wpilibmocks.ICompressor;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * The compressor component class describes the electronics of the compressor and defines the abstract way to control it.
 * The electronics include just a compressor. 
 *  
 * @author Will
 *
 */
@Singleton
public class CompressorComponent
{
    private final ICompressor compressor;

    /**
     * Initializes a new CompressorComponent
     */
    @Inject
    public CompressorComponent(ICompressor compressor)
    {
        this.compressor = compressor;
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
