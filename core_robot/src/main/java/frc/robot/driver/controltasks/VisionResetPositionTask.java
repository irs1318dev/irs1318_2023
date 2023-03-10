
//Calvin's experimental vision reset task

package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.common.robotprovider.Alliance;
import frc.robot.common.robotprovider.IDriverStation;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionResetPositionTask extends ControlTaskBase {

    private double[][] aprilTagLocations;
    private OffboardVisionManager visionManager;

    private Double xPosition;
    private Double yPosition;

    public VisionResetPositionTask()
    {
    }

    @Override
    public void begin()
    {
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);
        IRobotProvider rp = this.getInjector().getInstance(IRobotProvider.class);
        IDriverStation ds = rp.getDriverStation();
        if (ds.getAlliance() == Alliance.Red)
        {
            this.aprilTagLocations = TuningConstants.AprilTagLocationsRed;
        }
        else
        {
            this.aprilTagLocations = TuningConstants.AprilTagLocationsBlue;
        }

        this.xPosition = null;
        this.yPosition = null;

        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, true);
    }

    @Override
    public void update()
    {
        Double tagXOffset = this.visionManager.getAprilTagXOffset();
        Double tagYOffset = this.visionManager.getAprilTagYOffset();
        Double tagYaw = this.visionManager.getAprilTagYaw();
        Integer tagId = this.visionManager.getAprilTagId();

        if (tagId != null)
        {
            double[] thisTag = this.aprilTagLocations[tagId - 1];
            double alpha = tagYaw;
            double beta = Helpers.atan2d(tagXOffset, tagYOffset);
            double hypoFieldAngle = thisTag[2] - alpha - beta;
            if (Math.abs(hypoFieldAngle) > 180)
            {
                hypoFieldAngle += hypoFieldAngle > 0 ? -360 : 360;
            }

            double distToTag = Math.sqrt(tagXOffset * tagXOffset + tagYOffset * tagYOffset);
            this.xPosition = thisTag[0] + Helpers.cosd(hypoFieldAngle) * distToTag;
            this.yPosition = thisTag[1] + Helpers.sind(hypoFieldAngle) * distToTag;

            this.setDigitalOperationState(DigitalOperation.DriveTrainResetXYPosition, true);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingXPosition, this.xPosition);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingYPosition, this.yPosition);
        }
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainResetXYPosition, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainStartingXPosition, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainStartingYPosition, 0.0);
        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.xPosition != null && this.yPosition != null;
    }
}
