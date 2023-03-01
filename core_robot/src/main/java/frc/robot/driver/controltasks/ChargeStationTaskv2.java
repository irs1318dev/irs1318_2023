//Jamie Hsieh, Calvin Rodrigue
//1.19.2023
//Assumes the robot is on the center platform, and adjusts position.
//Jamie's first robot code commit.

package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.FloatingAverageCalculator;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

public class ChargeStationTaskv2 extends ControlTaskBase
{
    private enum State
    {
        Starting,
        Climbing,
        Balancing,
        Completed
    }

    private final double orientation;

    private State currentState;
    private FloatingAverageCalculator pitchRateAverageCalculator; 
    private PigeonManager imuManager;
    private ITimer timer;

    private double pitchRateAverage;
    private double pitch;
    private double climbingExceededTransitionTime;

    private final double reverse;

    public ChargeStationTaskv2(boolean reverse)
    {
        this(reverse, 0.0);
    }

    public ChargeStationTaskv2(boolean reverse, double orientation)
    {
        this.orientation = orientation;
        this.reverse = reverse ? -1.0 : 1.0;

        this.currentState = State.Starting;
        this.pitch = 0.0;
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

        //calculate floating average of past 0.1 seconds, at an expected 50 samples per second
        this.pitchRateAverageCalculator = new FloatingAverageCalculator(this.timer, 0.25, 50);
        this.pitch = this.imuManager.getPitch();

        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, true);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, this.orientation);
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
        this.pitchRateAverage = this.pitchRateAverageCalculator.update(this.imuManager.getPitchRate());

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
            if ((Math.abs(this.pitchRateAverage) <= TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF_V2) &&
                (Math.abs(this.pitch) <= TuningConstants.CHARGE_STATION_PITCH_VARIATION_V2))
            {
                this.currentState = State.Completed;
            }
        }

        System.out.println(this.currentState);
        switch (this.currentState)
        {
            case Balancing:
                // if the pitch diff over the past 0.1 seconds is greater than the acceptable diff
                if (Math.abs(this.pitchRateAverage) <= TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF_V2)
                {
                    // if negative pitch, move forward
                    if (this.pitch < 0)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_BALANCING_SPEED_V2);
                    }
                    else // if (this.pitch > 0)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -TuningConstants.CHARGE_STATION_BALANCING_SPEED_V2);
                    }
                }
                else // if pitch diff is within acceptable range, then pause.
                {
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                }

                break;

            case Climbing:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_CLIMBING_SPEED_V2 * this.reverse);
                break;

            case Starting:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_STARTING_SPEED_V2 * this.reverse);
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