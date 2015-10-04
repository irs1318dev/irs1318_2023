package org.usfirst.frc.team1318.robot.Compressor;

import edu.wpi.first.wpilibj.Compressor;

/**
 * The compressor component class describes the electronics of the compressor and defines the abstract way to control it.
 * The electronics include a compressor, and an analog pressure sensor. 
 *  
 * @author Will
 *
 */
public class CompressorComponent
{
    private final Compressor compressor;

    //    private AnalogInput analogPressureSensor;
    //    private DigitalInput digitalPressureSensor;

    /**
     * Initializes a new CompressorComponent
     */
    public CompressorComponent()
    {
        this.compressor = new Compressor();
        //
        //        this.digitalPressureSensor = new DigitalInput(0);

        //        this.analogPressureSensor = new AnalogInput(
        //            ElectronicsConstants.COMPRESSOR_ANALOG_PRESSURE_SENSOR);
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

    /**
     * Get the PSI in the pneumatic system
     * @return the current PSI
     */
    //    public double getPSI()
    //    {
    //        double psi = this.analogPressureSensor.getVoltage()
    //            * (ElectronicsConstants.COMPRESSOR_MAX_PSI / ElectronicsConstants.COMPRESSOR_MAX_VOLTAGE);
    //
    //        return psi;
    //    }
}
