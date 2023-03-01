//Jamie Hsieh, Calvin Rodrigue
//1.19.2023
//Assumes the robot is on the center platform, and adjusts position.
//Jamie's first robot code commit.

package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

public class ChargeStationTask extends ControlTaskBase
{
    public enum State
    {
        Starting,
        Climbing,
        Balancing,
        Completed
    }

    public State currentState;

    private PigeonManager imuManager;
    private ITimer timer;

    private double pitch;
    private double climbingExceededTransitionTime;

    private final double[] pitchLog;
    private double prevPitchLogTime;

    private double reverse = 1.0;

    public ChargeStationTask(boolean reverse)
    {
        this.reverse = 1.0;
        if (reverse)
        {
            this.reverse = -1.0;
        }

        this.currentState = State.Starting;
        this.pitch = 0.0;
        this.climbingExceededTransitionTime = 0.0;

        this.pitchLog = new double[25]; // logs every 0.02 secs, array is 0.5 secs total
        this.prevPitchLogTime = 0.0;
    }

    /**
     * finds the absolute value diff of pitch values of beginning and end of pitchLog[]
     * @return diff of first and last sample
     */
    private double findDiff()
    {
        return Math.abs((this.pitchLog[this.pitchLog.length - 1]) - this.pitchLog[0]);
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

        // at the beginning of task set all array values to be current pitch
        for (int i = 0; i < this.pitchLog.length; i++)
        {
            this.pitchLog[i] = this.pitch;
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, true);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.0);
        this.setDigitalOperationState(DigitalOperation.DriveTrainIgnoreSlewRateLimitingMode, true);
    }

    /**
     * Run an iteration of the current task.
     */
    @Override
    public void update()
    {
        double currTime = this.timer.get();

        this.pitch = this.imuManager.getPitch();

        // if previous pitch log time was at least 0.02 secs ago then set pitch log time as current time, and log the new pitch
        if (currTime - this.prevPitchLogTime >= 0.02)
        {
            this.prevPitchLogTime = currTime; 

            // constantly update array for past 0.5 seconds
            for (int i = 1; i < this.pitchLog.length; i++)
            {
                this.pitchLog[i - 1] = this.pitchLog[i];
            }

            // put current pitch in final space
            this.pitchLog[this.pitchLog.length - 1] = this.pitch;
        }

        if (this.currentState == State.Starting)
        {
            // if front wheel is on first part, set switch to climbing
            if (Math.abs(this.pitch) >= TuningConstants.CHARGE_STATION_START_TRANSITION_PITCH)
            {
                this.currentState = State.Climbing;
            }
        }
        else if (this.currentState == State.Climbing)
        {
            // if pitch is larger than 15-ish for more than the configured length of time, switch to balancing mode
            // TODO: Tune transition wait period on 2023 robot
            if (Math.abs(this.pitch) >= (TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_PITCH - TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_ACCEPTABLE_VARIATION) &&
                this.climbingExceededTransitionTime == 0.0)
            {
                this.climbingExceededTransitionTime = currTime;
            }

            if (this.climbingExceededTransitionTime != 0.0)
            {
                if (currTime - this.climbingExceededTransitionTime >= TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_WAIT_DURATION)
                {
                    this.currentState = State.Balancing;
                }
            }
        }
        else if (this.currentState == State.Balancing)
        {
            if ((TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF >= findDiff()) && 
                (Math.abs(this.pitch) <= TuningConstants.CHARGE_STATION_PITCH_VARIATION))
            {
                this.currentState = State.Completed;
            }
        }

        switch (this.currentState)
        {
            case Balancing:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_BALANCING_SPEED);

                // if the pitch diff over the past 0.5 seconds is greater than the acceptable diff
                if (TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF >= findDiff())
                {
                    // if negative pitch, move forward
                    if (this.pitch < 0)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_BALANCING_SPEED);
                    }
                    else // if (this.pitch > 0)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -TuningConstants.CHARGE_STATION_BALANCING_SPEED);
                    }
                }
                else // if pitch diff is within acceptable range, then pause.
                {
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                }

                break;

            case Climbing:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_CLIMBING_SPEED * this.reverse);
                break;

            case Starting:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_STARTING_SPEED * this.reverse);
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
        this.setDigitalOperationState(DigitalOperation.DriveTrainIgnoreSlewRateLimitingMode, false);
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