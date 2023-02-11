//Jamie Hsieh, Calvin Rodrigue
//1.19.2023
//Assumes the robot is on the center platform, and adjusts position.
//Jamie's first robot code commit.

package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

public class ChargeStationTaskGyro2 extends ControlTaskBase
{
    private enum State
    {
        Starting,
        Climbing,
        Balancing,
        Completed
    }

    private State currentState;

    private PigeonManager imuManager;
    private ITimer timer;

    private double pitch;
    private double pitchRate;
    private double climbingExceededTransitionTime;

    private double reverse = 1.0;

    public ChargeStationTaskGyro2(boolean reverse)
    {
        this.reverse = 1.0;
        if (reverse)
        {
            this.reverse = -1.0;
        }

        this.currentState = State.Starting;
        this.pitch = 0.0;
        this.pitchRate = 0.0;
        this.climbingExceededTransitionTime = 0.0;
    }

    /**
     * Begin the current task.
     */
    @Override
    public void begin()
    {
        this.currentState = State.Starting;

        this.imuManager = this.getInjector().getInstance(PigeonManager.class);
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.pitch = this.imuManager.getPitch();
        this.pitchRate = this.imuManager.getPitchRate();

        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, true);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.0);
    }

    /**
     * Run an iteration of the current task.
     */
    @Override
    public void update()
    {
        double currTime = this.timer.get();

        this.pitch = this.imuManager.getPitch();
        this.pitchRate = this.imuManager.getPitchRate();

        if (this.currentState == State.Starting)
        {
            // if front wheel is on first part, set switch to climbing
            if (Math.abs(this.pitch) >= TuningConstants.CHARGE_STATION_2_START_TRANSITION_PITCH)
            {
                this.currentState = State.Climbing;
            }
        }
        else if (this.currentState == State.Climbing)
        {
            // if pitch is larger than 15-ish for more than the configured length of time, switch to balancing mode
            if (this.reverse != -1.0 && this.pitchRate >= TuningConstants.CHARGE_STATION_2_TRANSITION_PITCH_DIFF ||
                this.reverse == -1.0 && this.pitchRate <= -TuningConstants.CHARGE_STATION_2_TRANSITION_PITCH_DIFF)
            {
                this.currentState = State.Balancing;
            }
        }
        else if (this.currentState == State.Balancing)
        {
            if ((Math.abs(this.pitchRate) <= TuningConstants.CHARGE_STATION_2_ACCEPTABLE_PITCH_DIFF) && 
                (Math.abs(this.pitch) <= TuningConstants.CHARGE_STATION_2_PITCH_VARIATION))
            {
                this.currentState = State.Completed;
            }
        }

        switch (this.currentState)
        {
            case Balancing:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_2_BALANCING_SPEED);

                // if the pitch diff over the past 0.5 seconds is greater than the acceptable diff
                if (Math.abs(this.pitchRate) <= TuningConstants.CHARGE_STATION_2_ACCEPTABLE_PITCH_DIFF)
                {
                    // if negative pitch, move forward
                    if (this.pitch < -5.0)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_2_FAST_BALANCING_SPEED);
                    }
                    else if (this.pitch < -0.5)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_2_BALANCING_SPEED);
                    }
                    else if (this.pitch > 5.0)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -TuningConstants.CHARGE_STATION_2_FAST_BALANCING_SPEED);
                    }
                    else if (this.pitch > 0.5)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -TuningConstants.CHARGE_STATION_2_BALANCING_SPEED);
                    }
                }
                else // if pitch diff is within acceptable range, then pause.
                {
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                }

                break;

            case Climbing:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_2_CLIMBING_SPEED * this.reverse);
                break;

            case Starting:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_2_STARTING_SPEED * this.reverse);
                break;

            default:
            case Completed:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                break;
        }
    }

    /**
     * Ends the current task, called when it (or a master task) has completed.
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed.
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return this.currentState == State.Completed;
    }
}