
//Calvin's experimental vision reset task

package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionResetPositionTask extends ControlTaskBase {

    private OffboardVisionManager visionManager;

    private Double tagXOffset;
    private Double tagYOffset;
    private Double tagYaw;
    private Integer tagId;

    private Double xPosition;
    private Double yPosition;

    public VisionResetPositionTask() {
        
    }

    @Override
    public void begin() {
        this.visionManager = this.getInjector().getInstance(OffboardVisionManager.class);

        this.setDigitalOperationState(DigitalOperation.VisionEnableRetroreflectiveProcessing, false);
        this.setDigitalOperationState(DigitalOperation.VisionEnableAprilTagProcessing, true);
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        this.tagXOffset = visionManager.getAprilTagXOffset();
        this.tagYOffset = visionManager.getAprilTagYOffset();
        this.tagYaw = visionManager.getAprilTagYaw();
        this.tagId = visionManager.getAprilTagId();
        this.calculatePosition();
        this.setPosition();
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        if (this.xPosition != null || this.yPosition != null)
        {
            this.setDigitalOperationState(DigitalOperation.DriveTrainResetXYPosition, false);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingXPosition, 0.0);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingYPosition, 0.0);
        }
    }

    @Override
    public boolean hasCompleted() {
        // TODO Auto-generated method stub
        return false;
    }

    private void calculatePosition() {
        if (visionManager.getAprilTagId() != null) {

            double[] thisTag = TuningConstants.AprilTagLocations[this.tagId - 1];
            double alpha = this.tagYaw;
            double beta = Math.atan2(this.tagXOffset, this.tagYOffset) * Helpers.RADIANS_TO_DEGREES;
            double hypoFieldAngle = thisTag[2] - alpha - beta;
            if (Math.abs(hypoFieldAngle) > 180) {
                hypoFieldAngle += hypoFieldAngle > 0 ? -360 : 360;
            }
            double distToTag = Math.sqrt(this.tagXOffset * this.tagXOffset + this.tagYOffset * this.tagYOffset);
            this.xPosition = thisTag[0] + Math.cos(Helpers.DEGREES_TO_RADIANS * hypoFieldAngle) * distToTag;
            this.yPosition = thisTag[1] + Math.sin(Helpers.DEGREES_TO_RADIANS * hypoFieldAngle) * distToTag;
            
        }
    }

    private void setPosition() {
        if (this.xPosition != null && this.yPosition != null) {
            this.setDigitalOperationState(DigitalOperation.DriveTrainResetXYPosition, true);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingXPosition, this.xPosition);
            this.setAnalogOperationState(AnalogOperation.DriveTrainStartingYPosition, this.yPosition);
        }
    }
    
}


