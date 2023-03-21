package frc.lib.robotprovider;

public interface IVisionListener<P extends IVisionPipeline>
{
    void copyPipelineOutputs(P pipeline);
}