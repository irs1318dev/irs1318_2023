package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;

public class HoverSubstationTask extends ControlTaskBase
{

    private double length;
    private double initialUpperArmPos;
    private double initialLowerArmPos;
    private double targetUpperArmPos;
    private double targetLowerArmPos;
    private double timeToTake;
    private double diffToTargetUpper;
    private double diffToTargetLower;

    @Override
    public void begin() {
        // set each point
        initialUpperArmPos = TuningConstants.ARM_UPPER_POSITION_INITIAL_HOVER_SUBSTATION;
        initialLowerArmPos = TuningConstants.ARM_LOWER_POSITION_INITIAL_HOVER_SUBSTATION;

        targetUpperArmPos = TuningConstants.ARM_UPPER_POSITION_TARGET_HOVER_SUBSTATION;
        targetLowerArmPos = TuningConstants.ARM_LOWER_POSITION_TARGET_HOVER_SUBSTATION;

        timeToTake = TuningConstants.HOVER_TASK_TIME_TO_TAKE;

        diffToTargetUpper = Math.abs(initialUpperArmPos - targetUpperArmPos);
        diffToTargetLower = Math.abs(initialLowerArmPos - targetLowerArmPos);
    }

    @Override
    public void update() {
        new ArmMMPositionTask(targetLowerArmPos, targetUpperArmPos);
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'end'");
    }

    @Override
    public boolean hasCompleted() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'hasCompleted'");
    }
    
}
