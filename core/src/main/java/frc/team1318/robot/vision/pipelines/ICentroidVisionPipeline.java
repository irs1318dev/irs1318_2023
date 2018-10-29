package frc.team1318.robot.vision.pipelines;

import frc.team1318.robot.common.robotprovider.*;

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
