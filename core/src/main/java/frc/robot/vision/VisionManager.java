package frc.robot.vision;

import frc.robot.ElectronicsConstants;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.Operation;
import frc.robot.driver.common.*;
import frc.robot.vision.common.*;
import frc.robot.vision.pipelines.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Vision manager.
 * 
 * @author Will
 *
 */
@Singleton
public class VisionManager implements IMechanism, IVisionListener<ICentroidVisionPipeline>
{
    private final static String LogName = "vision";

    private final IDashboardLogger logger;
    private final ITimer timer;
    private final ISolenoid ringLight;

    private final Object visionLock;

    private final IUsbCamera camera;
    private final Thread visionThread;
    private final HSVCenterPipeline visionPipeline;

    private Driver driver;
    private VisionProcessingState currentState;

    private IPoint center;

    private Double desiredAngleX;
    private Double measuredAngleX;
    private Double distanceFromRobot;

    private double lastMeasuredFps;

    /**
     * Initializes a new VisionManager
     * @param logger to use
     * @param timer to use
     * @param provider for obtaining electronics objects
     */
    @Inject
    public VisionManager(
        IDashboardLogger logger,
        ITimer timer,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.timer = timer;
        this.ringLight = provider.getSolenoid(ElectronicsConstants.PCM_A_MODULE, ElectronicsConstants.VISION_RING_LIGHT_PCM_CHANNEL);

        this.visionLock = new Object();

        this.camera = provider.getUsbCamera("usb0", 0);
        this.camera.setResolution(VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);

        this.camera.setExposureAuto();
        this.camera.setBrightness(VisionConstants.LIFECAM_CAMERA_OPERATOR_BRIGHTNESS);
        this.camera.setFPS(VisionConstants.LIFECAM_CAMERA_FPS);

        this.visionPipeline = new HSVCenterPipeline(this.timer, provider, VisionConstants.SHOULD_UNDISTORT);
        this.visionThread = this.camera.createVisionThread(this, this.visionPipeline);
        this.visionThread.start();

        this.driver = null;
        this.currentState = VisionProcessingState.None;

        this.center = null;
        this.desiredAngleX = null;
        this.measuredAngleX = null;
        this.distanceFromRobot = null;

        this.lastMeasuredFps = 0.0;
    }

    public IPoint getCenter()
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

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        IPoint center = this.getCenter();
        Double fps = this.getLastMeasuredFps();
        Double dist = this.getMeasuredDistance();
        Double dAngle = this.getDesiredAngle();
        Double mAngle = this.getMeasuredAngle();

        this.logger.logPoint(VisionManager.LogName, "center", center);
        this.logger.logNumber(VisionManager.LogName, "fps", fps);
        this.logger.logNumber(VisionManager.LogName, "dist", dist);
        this.logger.logNumber(VisionManager.LogName, "dAngle", dAngle);
        this.logger.logNumber(VisionManager.LogName, "mAngle", mAngle);
    }

    @Override
    public void update()
    {
        VisionProcessingState desiredState = VisionProcessingState.None;
        if (this.driver.getDigital(Operation.EnableVision))
        {
            desiredState = VisionProcessingState.Active;
        }

        if (this.currentState != desiredState)
        {
            if (desiredState == VisionProcessingState.Active)
            {
                this.camera.setExposureManual(VisionConstants.LIFECAM_CAMERA_VISION_EXPOSURE);
                this.camera.setBrightness(VisionConstants.LIFECAM_CAMERA_VISION_BRIGHTNESS);
                this.camera.setFPS(VisionConstants.LIFECAM_CAMERA_FPS);
            }
            else
            {
                this.camera.setExposureAuto();
                this.camera.setBrightness(VisionConstants.LIFECAM_CAMERA_OPERATOR_BRIGHTNESS);
                this.camera.setFPS(VisionConstants.LIFECAM_CAMERA_FPS);
            }

            this.ringLight.set(desiredState == VisionProcessingState.Active);
            this.visionPipeline.setActivation(desiredState == VisionProcessingState.Active);

            this.currentState = desiredState;
        }
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
