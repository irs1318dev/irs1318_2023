package frc.robot.driver.controltasks;

import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;

public class IntakeTask extends ControlTaskBase
{
    private ArmMechanism arm;
    private ITimer timer;
    private double duration;
    private double startTime;
    private boolean inTimeMode;
    public IntakeTask()
    {
        inTimeMode = false;
    }

    public IntakeTask(double duration)
    {
        this.duration = duration;
        inTimeMode = true;
    }



    @Override
    public void begin() {
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        this.setDigitalOperationState(DigitalOperation.IntakeIn, true);
        this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
        this.setDigitalOperationState(DigitalOperation.IntakeExtend, true);
        this.setDigitalOperationState(DigitalOperation.IntakeRetract, false);
        startTime = this.timer.get();
    }

    @Override
    public void update() {
        this.setDigitalOperationState(DigitalOperation.IntakeIn, true);
        this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
        this.setDigitalOperationState(DigitalOperation.IntakeExtend, true);
        this.setDigitalOperationState(DigitalOperation.IntakeRetract, false);
        
    }

    @Override
    public void end() {
        this.setDigitalOperationState(DigitalOperation.IntakeIn, false);
        this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
        this.setDigitalOperationState(DigitalOperation.IntakeExtend, false);
        this.setDigitalOperationState(DigitalOperation.IntakeRetract, false);
        
    }

    @Override
    public boolean hasCompleted() {
        if (inTimeMode)
        {
            if (this.timer.get() >= startTime + duration)
            {
                return true;
            }
        }
        return this.arm.getLightValue();
    }
    
}

