package frc.robot.mechanisms;

import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;
import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.*;
import frc.robot.vision.*;
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
    private final IMotor ringLight;

    private final Object visionLock;

    private final IUsbCamera camera;
    private final Thread visionThread;
    private final ICentroidVisionPipeline visionPipeline;

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
        this.ringLight = provider.getTalon(ElectronicsConstants.VISION_RING_LIGHT_PWM_CHANNEL);

        this.visionLock = new Object();

        this.camera = provider.getUsbCamera("usb0", 0);
        this.camera.setResolution(VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);

        this.camera.setExposureAuto();
        this.camera.setBrightness(VisionConstants.LIFECAM_CAMERA_OPERATOR_BRIGHTNESS);
        this.camera.setFPS(VisionConstants.LIFECAM_CAMERA_FPS);

        this.visionPipeline = new HSVDockingCenterRectanglePipeline(this.timer, provider, VisionConstants.SHOULD_UNDISTORT);
        this.visionThread = this.camera.createVisionThread(this, this.visionPipeline);
        this.visionThread.start();

        this.driver = null;
        this.currentState = VisionProcessingState.Disabled;

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

    public VisionProcessingState getState()
    {
        synchronized (this.visionLock)
        {
            return this.currentState;
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
        VisionProcessingState desiredState = this.currentState;
        if (this.driver.getDigital(DigitalOperation.VisionEnable))
        {
            desiredState = VisionProcessingState.Active;
        }
        else if (this.driver.getDigital(DigitalOperation.VisionDisable))
        {
            desiredState = VisionProcessingState.Disabled;
        }

        if (this.driver.getDigital(DigitalOperation.VisionForceDisable))
        {
            desiredState = VisionProcessingState.Disabled;
        }
        else if (TuningConstants.VISION_ENABLE_DURING_TELEOP &&
            !this.driver.isAutonomous() &&
            desiredState == VisionProcessingState.Disabled)
        {
            desiredState = VisionProcessingState.Active;
        }

        if (this.currentState != desiredState)
        {
            this.changeState(desiredState);
        }

        // vision pipeline should only write frames to the stream when we are not having the offboard
        // vision system do streaming
        boolean enableOffboardStream = this.driver.getDigital(DigitalOperation.VisionEnableOffboardStream);
        this.visionPipeline.setStreamMode(!enableOffboardStream);
    }

    @Override
    public void stop()
    {
        this.ringLight.set(0.0);
        this.visionPipeline.setMode(VisionProcessingState.Disabled);
        this.visionPipeline.setStreamMode(true);
        this.currentState = VisionProcessingState.Disabled;

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

        if (TuningConstants.VISION_ENABLE_DURING_TELEOP &&
            !this.driver.isAutonomous() &&
            this.currentState != VisionProcessingState.Active)
        {
            this.changeState(VisionProcessingState.Active);
        }
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
            else
            {
                this.center = null;
                this.desiredAngleX = null;
                this.measuredAngleX = null;
                this.distanceFromRobot = null;
                this.lastMeasuredFps = 0.0;
            }
        }
    }

    private void changeState(VisionProcessingState desiredState)
    {
        boolean hsvMode = desiredState == VisionProcessingState.Active;

        if (hsvMode)
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

        this.ringLight.set(hsvMode ? VisionConstants.RING_LIGHT_ON : VisionConstants.RING_LIGHT_OFF);
        this.visionPipeline.setMode(desiredState);

        this.currentState = desiredState;
    }
}
