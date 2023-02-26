package frc.robot.driver.controltasks;

import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.DigitalOperation;
import frc.robot.mechanisms.ArmMechanism;

public class IntakeGamePieceTask extends ControlTaskBase
{
    private enum IntakeState
    {
        Intake,
        Close,
        Completed
    }

    private final double maxDuration;
    private final boolean useTimeout;

    private ArmMechanism arm;
    private ITimer timer;

    private double startTime;
    private IntakeState state;

    /**
     * Intake until the button is released or after the through-beam senses the game piece
     */
    public IntakeGamePieceTask()
    {
        this.useTimeout = false;
        this.maxDuration = 0.0;
        this.state = IntakeState.Intake;
    }

    /**
     * Intake until the button is released or after the through-beam senses the game piece
     * @param maxDuration we will intake before giving up
     */
    public IntakeGamePieceTask(double maxDuration)
    {
        this.maxDuration = maxDuration;
        this.useTimeout = true;
        this.state = IntakeState.Intake;
    }

    @Override
    public void begin()
    {
        this.arm = this.getInjector().getInstance(ArmMechanism.class);
        if (this.useTimeout)
        {
            this.timer = this.getInjector().getInstance(ITimer.class);
            this.startTime = this.timer.get();
        }

        this.setDigitalOperationState(DigitalOperation.IntakeIn, true);
        this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
        this.setDigitalOperationState(DigitalOperation.IntakeRelease, true);
        this.setDigitalOperationState(DigitalOperation.IntakeGrab, false);
    }

    @Override
    public void update()
    {
        if (this.state == IntakeState.Intake &&
            this.arm.isThroughBeamBroken())
        {
            this.state = IntakeState.Close;
        }
        else if (this.state == IntakeState.Close)
        {
            this.state = IntakeState.Completed;
        }
        else if (this.useTimeout)
        {
            if (this.timer.get() >= this.startTime + this.maxDuration)
            {
                this.state = IntakeState.Completed;
            }
        }

        switch (this.state)
        {
            case Intake:
                this.setDigitalOperationState(DigitalOperation.IntakeIn, true);
                this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
                this.setDigitalOperationState(DigitalOperation.IntakeRelease, true);
                this.setDigitalOperationState(DigitalOperation.IntakeGrab, false);
                break;

            case Close:
                this.setDigitalOperationState(DigitalOperation.IntakeIn, false);
                this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
                this.setDigitalOperationState(DigitalOperation.IntakeRelease, false);
                this.setDigitalOperationState(DigitalOperation.IntakeGrab, true);
                break;

            default:
            case Completed:
                this.setDigitalOperationState(DigitalOperation.IntakeIn, false);
                this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
                this.setDigitalOperationState(DigitalOperation.IntakeRelease, false);
                this.setDigitalOperationState(DigitalOperation.IntakeGrab, false);
                break;
        }
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.IntakeIn, false);
        this.setDigitalOperationState(DigitalOperation.IntakeOut, false);
        this.setDigitalOperationState(DigitalOperation.IntakeRelease, false);
        this.setDigitalOperationState(DigitalOperation.IntakeGrab, false);
    }

    @Override
    public boolean hasCompleted()
    {
        return this.arm.isThroughBeamBroken();
    }
}

