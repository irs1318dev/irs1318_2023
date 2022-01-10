package frc.robot.common.robotprovider;

import edu.wpi.first.wpilibj.Compressor;

public class CompressorWrapper implements ICompressor
{
    private final Compressor wrappedObject;

    public CompressorWrapper(PneumaticsModuleType moduleType)
    {
        this.wrappedObject = new Compressor(CompressorWrapper.getModuleType(moduleType));
    }

    public CompressorWrapper(int module, PneumaticsModuleType moduleType)
    {
        this.wrappedObject = new Compressor(module, CompressorWrapper.getModuleType(moduleType));
    }

    static edu.wpi.first.wpilibj.PneumaticsModuleType getModuleType(PneumaticsModuleType moduleType)
    {
        if (moduleType == PneumaticsModuleType.PneumaticsControlModule)
        {
            return edu.wpi.first.wpilibj.PneumaticsModuleType.CTREPCM;
        }
        else // if (moduleType == PneumaticsModuleType.PneumaticsHub)
        {
            return edu.wpi.first.wpilibj.PneumaticsModuleType.REVPH;
        }
    }

    public void enableAnalog(double minPressurePSI, double maxPressurePSI)
    {
        this.wrappedObject.enableAnalog(minPressurePSI, maxPressurePSI);
    }

    public void enableDigital()
    {
        this.wrappedObject.enableDigital();
    }

    public void disable()
    {
        this.wrappedObject.disable();
    }
}
