package frc.team1318.robot.common.robotprovider;

import org.opencv.core.Mat;

public interface IVisionPipeline
{
    /**
     * Processes the image input and sets the result objects.
     * Implementations should make these objects accessible.
     */
    void process(Mat image);
}