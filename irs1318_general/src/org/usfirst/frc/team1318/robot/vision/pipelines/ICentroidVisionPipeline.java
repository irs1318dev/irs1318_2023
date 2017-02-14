package org.usfirst.frc.team1318.robot.vision.pipelines;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

public interface ICentroidVisionPipeline extends VisionPipeline
{
    void setActivation(boolean active);
    Point getCenter();
    Double getDesiredAngleX();
    Double getMeasuredAngleX();
    Double getRobotDistance();
    double getFps();
}
