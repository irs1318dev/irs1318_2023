package org.usfirst.frc.team1318.robot.Compressor;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;
import edu.wpi.first.wpilibj.*;

/**
 * The compressor component class describes the electronics of the compressor and defines the abstract way to control it.
 * The electronics include a compressor, and an analog pressure sensor. 
 *  
 * @author Will
 *
 */
public class CompressorComponent
{
    // logging constants
    private static final String RUNNING_LOG_KEY = "cm.r";
    private static final String PSI_LOG_KEY = "cm.p";

    private Compressor compressor;
    private AnalogInput analogPressureSensor;

    /**
     * Initializes a new CompressorComponent
     */
    public CompressorComponent()
    {
        this.compressor = new Compressor();

        this.analogPressureSensor = new AnalogInput(
            ElectronicsConstants.COMPRESSOR_ANALOG_PRESSURE_SENSOR);
    }

    /**
     * Start the compressor 
     */
    public void start()
    {
        this.compressor.start();

        SmartDashboardLogger.putBoolean(CompressorComponent.RUNNING_LOG_KEY, true);
    }

    /**
     * Stop the compressor
     */
    public void stop()
    {
        this.compressor.stop();

        SmartDashboardLogger.putBoolean(CompressorComponent.RUNNING_LOG_KEY, false);
    }

    /**
     * Get the PSI in the pneumatic system
     * @return the current PSI
     */
    public double getPSI()
    {
        double psi = this.analogPressureSensor.getVoltage() * (ElectronicsConstants.COMPRESSOR_MAX_PSI / ElectronicsConstants.COMPRESSOR_MAX_VOLTAGE);

        SmartDashboardLogger.putNumber(CompressorComponent.PSI_LOG_KEY, psi);

        return psi;
    }
}
