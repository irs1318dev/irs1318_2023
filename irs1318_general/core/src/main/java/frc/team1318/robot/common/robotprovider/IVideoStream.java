package frc.team1318.robot.common.robotprovider;

import org.opencv.core.Mat;

public interface IVideoStream
{
    void putFrame(Mat image);
}