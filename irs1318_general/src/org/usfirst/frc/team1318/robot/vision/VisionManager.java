package org.usfirst.frc.team1318.robot.vision;

import org.opencv.core.Point;
import org.usfirst.frc.team1318.robot.common.DashboardLogger;
import org.usfirst.frc.team1318.robot.common.IController;
import org.usfirst.frc.team1318.robot.driver.Driver;
import org.usfirst.frc.team1318.robot.vision.analyzer.HSVCenterPipeline;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * Vision manager.
 * 
 * @author Will
 *
 */
public class VisionManager implements IController, VisionRunner.Listener<HSVCenterPipeline>
{
    private final static String LogName = "vision";

    private final Object visionLock;
    private final VisionThread visionThread;

    private Point center1;
    private Point center2;
    private double lastMeasuredFps;

    /**
     * Initializes a new VisionManager
     */
    public VisionManager()
    {
        this.visionLock = new Object();
        AxisCamera camera = CameraServer.getInstance().addAxisCamera(VisionConstants.CAMERA_IP_ADDRESS);
        this.visionThread = new VisionThread(camera, new HSVCenterPipeline(), this);
        this.visionThread.start();

        this.center1 = null;
        this.center2 = null;
        this.lastMeasuredFps = 0.0;
    }

    public Point getCenter1()
    {
        synchronized (this.visionLock)
        {
            return this.center1;
        }
    }

    public Double getCenter1Angle()
    {
        Point center1 = this.getCenter1();
        if (center1 != null)
        {
            // note: positive angle means it is to the right
            double centerX = center1.x;
            centerX = centerX - VisionConstants.CAMERA_CENTER_WIDTH;
            return (centerX * VisionConstants.CAMERA_CENTER_VIEW_ANGLE) / (double)VisionConstants.CAMERA_CENTER_WIDTH;
        }

        return null;
    }

    public Point getCenter2()
    {
        synchronized (this.visionLock)
        {
            return this.center2;
        }
    }

    public double getLastMeasuredFps()
    {
        synchronized (this.visionLock)
        {
            return this.lastMeasuredFps;
        }
    }

    @Override
    public void update()
    {
        String center1String = "n/a";
        Point center1 = this.getCenter1();
        if (center1 != null)
        {
            center1String = String.format("%f,%f", center1.x, center1.y);
        }

        DashboardLogger.logString(VisionManager.LogName, "center1", center1String);

        String center1AngleString = "n/a";
        Double centerAngle = this.getCenter1Angle();
        if (centerAngle != null)
        {
            center1AngleString = String.format("%f", centerAngle);
        }

        DashboardLogger.logString(VisionManager.LogName, "center1Angle", center1AngleString);

        String center2String = "n/a";
        Point center2 = this.getCenter2();
        if (center2 != null)
        {
            center2String = String.format("%f,%f", center2.x, center2.y);
        }

        DashboardLogger.logString(VisionManager.LogName, "center2", center2String);

        double fps = this.getLastMeasuredFps();
        DashboardLogger.logNumber(VisionManager.LogName, "fps", fps);
    }

    @Override
    public void stop()
    {
    }

    @Override
    public void setDriver(Driver driver)
    {
        // no-op
    }

    @Override
    public void copyPipelineOutputs(HSVCenterPipeline pipeline)
    {
        synchronized (this.visionLock)
        {
            this.center1 = pipeline.getCenter1();
            this.center2 = pipeline.getCenter2();
            this.lastMeasuredFps = pipeline.getFps();
        }
    }
}
