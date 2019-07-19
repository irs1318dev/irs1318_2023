package frc.robot.mechanisms;

import frc.robot.common.*;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 * 
 * @author Will
 *
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    private static final String logName = "rpi";

    private final INetworkTableProvider networkTable;
    private final IDashboardLogger logger;

    private Driver driver;

    private double ballCenterX;
    private double ballCenterY;
    private double ballDistance;
    private String ballDirection;

    /**
     * Initializes a new OffboardVisionManager
     * @param provider for obtaining electronics objects
     * @param logger for logging to smart dashboard
     */
    @Inject
    public OffboardVisionManager(IRobotProvider provider, IDashboardLogger logger)
    {
        this.networkTable = provider.getNetworkTableProvider();
        this.logger = logger;

        this.ballCenterX = 0.0;
        this.ballCenterY = 0.0;
        this.ballDistance = 0.0;
        this.ballDirection = "";
    }

    public double getBallCenterX()
    {
        return this.ballCenterX;
    }

    public double getBallCenterY()
    {
        return this.ballCenterY;
    }

    public double getBallDistance()
    {
        return this.ballDistance;
    }

    public String getBallDirection()
    {
        return this.ballDirection;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.ballCenterX = this.networkTable.getSmartDashboardNumber("rpi.BallcenterX");
        this.ballCenterY = this.networkTable.getSmartDashboardNumber("rpi.BallcenterY");
        this.ballDistance = this.networkTable.getSmartDashboardNumber("rpi.BallDistance");
        this.ballDirection = this.networkTable.getSmartDashboardString("rpi.BallDirection");
    }

    @Override
    public void update()
    {
        boolean enableVideoStream = this.driver.getDigital(DigitalOperation.VisionEnableOffboardStream);
        boolean enableVideoProcessing = this.driver.getDigital(DigitalOperation.VisionEnableOffboardProcessing);
        this.logger.logBoolean(OffboardVisionManager.logName, "enableStream", enableVideoStream);
        this.logger.logBoolean(OffboardVisionManager.logName, "enableProcessing", enableVideoProcessing);
    }

    @Override
    public void stop()
    {
        this.logger.logBoolean(OffboardVisionManager.logName, "enableStream", false);
        this.logger.logBoolean(OffboardVisionManager.logName, "enableProcessing", false);
    }

    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }
}
