package frc.robot.vision.pipelines;

import frc.robot.common.robotprovider.*;

public interface ICentroidVisionPipeline extends IVisionPipeline
{
    void setActivation(boolean active);
    boolean isActive();
    IPoint getCenter();
    Double getDesiredAngleX();
    Double getMeasuredAngleX();
    Double getRobotDistance();
    double getFps();
}
