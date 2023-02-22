/* 
//Calvin's experimental vision reset task

package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.OffboardVisionManager;

public class VisionResetPositionTask extends ControlTaskBase {

    private OffboardVisionManager visionManager;

    private Double tagXOffset;
    private Double tagYOffset;
    private Double tagYaw;
    private int tagId;

    private final Double xPosition;
    private final Double yPosition;
    private final Double orientationAngle;

    public VisionResetPositionTask() {
        this.xPosition = 0.0;
        this.yPosition = 0.0;
        this.orientationAngle = 0.0;
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
        this.setPosition();
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        this.setPosition();
    }

    @Override
    public boolean hasCompleted() {
        // TODO Auto-generated method stub
        return false;
    }

    private void setPosition() {
        if (visionManager.getAprilTagId() != null) {
            double[] thisTag = TuningConstants.AprilTagLocations[this.tagId];
            double robotFieldOrientation = (-thisTag[2] - this.tagYaw);
            if (Math.abs(robotFieldOrientation) > 180) {
                robotFieldOrientation += robotFieldOrientation < 0 ? 360 : -360;
            }

            if (thisTag[2] == 0.0) {//tag facing red alliance

            }
            double distToTag = Math.sqrt(this.tagXOffset * this.tagXOffset + this.tagYOffset * this.tagYOffset);
            double newRobotX = 0;//hisTag[0] + Math.cos(Helpers.DEGREES_TO_RADIANS * angleToRobot) * distToTag;
            double newRobotY = 0;//thisTag[0] + Math.sin(Helpers.DEGREES_TO_RADIANS * angleToRobot) * distToTag;

        }
    }
    
}

*/
