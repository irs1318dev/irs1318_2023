package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;

public class HoverSubstationTask extends ControlTaskBase
{

    private double initialUpperArmPos;
    private double initialLowerArmPos;

    private double stageOneUpperArmPos;
    private double stageOneLowerArmPos;

    private double stageTwoUpperArmPos;
    private double stageTwoLowerArmPos;

    private double timeToTake;
    private double diffToTargetUpper;
    private double diffToTargetLower;

    @Override
    public void begin() {
        // set each point
        initialUpperArmPos = TuningConstants.ARM_UPPER_POSITION_INITIAL_HOVER_SUBSTATION;
        initialLowerArmPos = TuningConstants.ARM_LOWER_POSITION_INITIAL_HOVER_SUBSTATION;

        stageOneUpperArmPos = TuningConstants.ARM_UPPER_POSITION_TARGET_HOVER_SUBSTATION_MIDPOINT;
        stageOneLowerArmPos = TuningConstants.ARM_LOWER_POSITION_TARGET_HOVER_SUBSTATION_MIDPOINT;  

        stageTwoUpperArmPos = TuningConstants.ARM_UPPER_POSITION_TARGET_HOVER_SUBSTATION_FINAL;
        stageTwoLowerArmPos = TuningConstants.ARM_LOWER_POSITION_TARGET_HOVER_SUBSTATION_FINAL;

        timeToTake = TuningConstants.HOVER_TASK_TIME_TO_TAKE;

        diffToTargetUpper = Math.abs(initialUpperArmPos - stageOneUpperArmPos);
        diffToTargetLower = Math.abs(initialLowerArmPos - stageOneLowerArmPos);
    }

    @Override
    public void update() {
        new ArmIKPositionTask(stageOneLowerArmPos, stageOneUpperArmPos);
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
