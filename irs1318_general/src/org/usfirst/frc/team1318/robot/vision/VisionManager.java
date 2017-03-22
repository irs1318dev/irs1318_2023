package org.usfirst.frc.team1318.robot.vision;

import org.opencv.core.Point;
import org.usfirst.frc.team1318.robot.common.IController;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.ISolenoid;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.ITimer;
import org.usfirst.frc.team1318.robot.driver.Driver;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.vision.pipelines.HSVCenterPipeline;
import org.usfirst.frc.team1318.robot.vision.pipelines.ICentroidVisionPipeline;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.google.inject.name.Named;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * Vision manager.
 * 
 * @author Will
 *
 */
@Singleton
public class VisionManager implements IController, VisionRunner.Listener<ICentroidVisionPipeline>
{
    private final static String LogName = "vision";

    private final IDashboardLogger logger;
    private final ITimer timer;
    private final ISolenoid ringLight;

    private final Object visionLock;

    private final VisionThread visionThread;
    private final HSVCenterPipeline visionPipeline;

    private Driver driver;

    private Point center;

    private Double desiredAngleX;
    private Double measuredAngleX;
    private Double distanceFromRobot;

    private double lastMeasuredFps;

    /**
     * Initializes a new VisionManager
     */
    @Inject
    public VisionManager(
        IDashboardLogger logger,
        ITimer timer,
        @Named("VISION_LIGHT") ISolenoid ringLight)
    {
        this.logger = logger;
        this.timer = timer;
        this.ringLight = ringLight;

        this.driver = null;

        this.visionLock = new Object();

        UsbCamera camera = new UsbCamera("usb1", 0);
        camera.setResolution(VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);
        camera.setExposureManual(VisionConstants.LIFECAM_CAMERA_EXPOSURE);
        camera.setBrightness(VisionConstants.LIFECAM_CAMERA_BRIGHTNESS);
        camera.setFPS(VisionConstants.LIFECAM_CAMERA_FPS);

        this.visionPipeline = new HSVCenterPipeline(this.timer, VisionConstants.SHOULD_UNDISTORT);
        this.visionThread = new VisionThread(camera, this.visionPipeline, this);
        this.visionThread.start();

        this.center = null;
        this.desiredAngleX = null;
        this.measuredAngleX = null;
        this.distanceFromRobot = null;

        this.lastMeasuredFps = 0.0;
    }

    public Point getCenter()
    {
        synchronized (this.visionLock)
        {
            return this.center;
        }
    }

    public Double getMeasuredAngle()
    {
        synchronized (this.visionLock)
        {
            return this.measuredAngleX;
        }
    }

    public Double getDesiredAngle()
    {
        synchronized (this.visionLock)
        {
            return this.desiredAngleX;
        }
    }

    public Double getMeasuredDistance()
    {
        synchronized (this.visionLock)
        {
            return this.distanceFromRobot;
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
        boolean cameraActive;
        if (this.driver.getDigital(Operation.EnableVision))
        {
            cameraActive = true;
        }
        else
        {
            cameraActive = false;
        }

        this.ringLight.set(cameraActive);
        this.visionPipeline.setActivation(cameraActive);

        Point center = this.getCenter();
        this.logger.logPoint(VisionManager.LogName, "center", center);

        Double fps = this.getLastMeasuredFps();
        this.logger.logNumber(VisionManager.LogName, "fps", fps);

        Double dist = this.getMeasuredDistance();
        this.logger.logNumber(VisionManager.LogName, "dist", dist);

        Double dAngle = this.getDesiredAngle();
        this.logger.logNumber(VisionManager.LogName, "dAngle", dAngle);

        Double mAngle = this.getMeasuredAngle();
        this.logger.logNumber(VisionManager.LogName, "mAngle", mAngle);
    }

    @Override
    public void stop()
    {
        this.ringLight.set(false);
        this.visionPipeline.setActivation(false);

        this.center = null;

        this.desiredAngleX = null;
        this.measuredAngleX = null;
        this.distanceFromRobot = null;

        this.lastMeasuredFps = 0.0;
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    @Override
    public void copyPipelineOutputs(ICentroidVisionPipeline pipeline)
    {
        synchronized (this.visionLock)
        {
            if (pipeline.isActive())
            {
                this.center = pipeline.getCenter();

                this.desiredAngleX = pipeline.getDesiredAngleX();
                this.measuredAngleX = pipeline.getMeasuredAngleX();
                this.distanceFromRobot = pipeline.getRobotDistance();

                this.lastMeasuredFps = pipeline.getFps();
            }
        }
    }
}
