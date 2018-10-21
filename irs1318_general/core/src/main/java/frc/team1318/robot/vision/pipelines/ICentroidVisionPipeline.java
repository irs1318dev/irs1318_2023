package frc.team1318.robot.vision.pipelines;

import org.opencv.core.Point;

import frc.team1318.robot.common.robotprovider.IVisionPipeline;


public interface ICentroidVisionPipeline extends IVisionPipeline
{
    void setActivation(boolean active);
    boolean isActive();
    Point getCenter();
    Double getDesiredAngleX();
    Double getMeasuredAngleX();
    Double getRobotDistance();
    double getFps();
}
