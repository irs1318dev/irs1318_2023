package frc.robot.driver.controltasks;

import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;

public class IntakeTask extends ControlTaskBase
{
    private final double duration;
    private final boolean inTimeMode;

    private ArmMechanism arm;
    private ITimer timer;

    private double startTime;

    public IntakeTask()
    {
        this.inTimeMode = false;
        this.duration = 0.0;
    }

    public IntakeTask(double duration)
    {
        this.duration = duration;
        this.inTimeMode = true;
    }

    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        if (this.inTimeMode)
        {
            this.timer = this.getInjector().getInstance(ITimer.class);
            this.startTime = this.timer.get();
        }

        this.setDigitalOperationState(DigitalOperation.IntakeIn, true);
        this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
        this.setDigitalOperationState(DigitalOperation.IntakeExtend, true);
        this.setDigitalOperationState(DigitalOperation.IntakeRetract, false);
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.IntakeIn, true);
        this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
        this.setDigitalOperationState(DigitalOperation.IntakeExtend, true);
        this.setDigitalOperationState(DigitalOperation.IntakeRetract, false);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.IntakeIn, false);
        this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
        this.setDigitalOperationState(DigitalOperation.IntakeExtend, false);
        this.setDigitalOperationState(DigitalOperation.IntakeRetract, false);
    }

    @Override
    public boolean hasCompleted()
    {
        if (this.inTimeMode)
        {
            if (this.timer.get() >= this.startTime + this.duration)
            {
                return true;
            }
        }

        return this.arm.isThroughBeamBroken();
    }
}

