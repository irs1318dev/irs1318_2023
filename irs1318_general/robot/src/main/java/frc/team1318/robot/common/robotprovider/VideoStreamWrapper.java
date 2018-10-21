package frc.team1318.robot.common.robotprovider;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;

public class VideoStreamWrapper implements IVideoStream
{
    public final CvSource wrappedObject;

    public VideoStreamWrapper(String name, int width, int height)
    {
        this.wrappedObject = CameraServer.getInstance().putVideo(name, width, height);
    }

    public void putFrame(Mat image)
    {
        this.wrappedObject.putFrame(image);
    }
}