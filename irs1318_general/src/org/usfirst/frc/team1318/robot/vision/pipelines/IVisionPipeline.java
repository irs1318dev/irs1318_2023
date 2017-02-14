package org.usfirst.frc.team1318.robot.vision.pipelines;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

public interface IVisionPipeline extends VisionPipeline
{
    void setActivation(boolean active);
}
