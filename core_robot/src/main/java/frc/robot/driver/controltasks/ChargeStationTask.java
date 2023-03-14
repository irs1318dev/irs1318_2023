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

public class ChargeStationTask extends ControlTaskBase
{
    public enum Orientation
    {
        Forwards,
        Backwards,
        Left,
        Right,
    }

    private enum State
    {
        Starting,
        Climbing,
        Balancing,
        Completed
    }

    private final double reverse;
    private final Orientation orientation;

    private PigeonManager imuManager;
    private ITimer timer;

    private FloatingAverageCalculator angleRateAverageCalculator; 
    private double angleRateAverage;

    private State currentState;
    private double angle;
    private double climbingExceededTransitionTime;

    /**
     * Charge station balancing task
     * @param reverse whether to travel towards own grid (true) or toward opponent grid (false) - field-relative
     */
    public ChargeStationTask(boolean reverse)
    {
        this(reverse, Orientation.Forwards);
    }

    /**
     * Charge station balancing task
     * @param reverse whether to travel towards own grid (true) or toward opponent grid (false) - field-relative
     * @param orientation which direction to face while balancing - forwards (away from our grid), left, right, or backwards (towards our grid)
     */
    public ChargeStationTask(boolean reverse, Orientation orientation)
    {
        this.orientation = orientation;
        this.reverse = reverse ? -1.0 : 1.0;
    }

    /**
     * Begin the current task.
     */
    @Override
    public void begin()
    {
        this.currentState = State.Starting;
        this.climbingExceededTransitionTime = 0.0;

        this.imuManager = this.getInjector().getInstance(PigeonManager.class);
        this.timer = this.getInjector().getInstance(ITimer.class);

        // calculate floating average of past 0.1 seconds
        this.angleRateAverage = 0.0;
        this.angleRateAverageCalculator = new FloatingAverageCalculator(this.timer, 0.25, 50);
        switch (this.orientation)
        {
            case Left:
            case Right:
                this.angle = this.imuManager.getRoll();
                break;

            default:
            case Forwards:
            case Backwards:
                this.angle = this.imuManager.getPitch();
                break;
        }

        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, true);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        switch (this.orientation)
        {
            case Backwards:
                this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, 180.0);
                break;
            case Left:
                this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, 90.0);
                break;
            case Right:
                this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, -90.0);
                break;
            default:
            case Forwards:
                this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, 0.0);
                break;
        }

        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.0);
        this.setDigitalOperationState(DigitalOperation.DriveTrainIgnoreSlewRateLimitingMode, false);
    }

    /**
     * Run an iteration of the current task.
     */
    @Override
    public void update()
    {
        double currTime = this.timer.get();

        switch (this.orientation)
        {
            case Left:
            case Right:
                this.angle = this.imuManager.getRoll();
                this.angleRateAverage = this.angleRateAverageCalculator.update(this.imuManager.getRollRate());
                break;

            default:
            case Forwards:
            case Backwards:
                this.angle = this.imuManager.getPitch();
                this.angleRateAverage = this.angleRateAverageCalculator.update(this.imuManager.getPitchRate());
                break;
        }

        if (this.currentState == State.Starting)
        {
            // if front wheel is on first part, set switch to climbing
            if (Math.abs(this.angle) >= TuningConstants.CHARGE_STATION_START_TRANSITION_PITCH)
            {
                this.currentState = State.Climbing;
            }
        }
        else if (this.currentState == State.Climbing)
        {
            // if pitch is larger than 15-ish for more than the configured length of time, switch to balancing mode
            if (Math.abs(this.angle) >= (TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_PITCH - TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_ACCEPTABLE_VARIATION) &&
                this.climbingExceededTransitionTime == 0.0)
            {
                this.climbingExceededTransitionTime = currTime;
            }

            if (this.climbingExceededTransitionTime != 0.0)
            {
                if (currTime - this.climbingExceededTransitionTime >= TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_WAIT_DURATION_V2)
                {
                    this.currentState = State.Balancing;
                }
            }
        }
        else if (this.currentState == State.Balancing)
        {
            if ((Math.abs(this.angleRateAverage) <= TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF_V2) &&
                (Math.abs(this.angle) <= TuningConstants.CHARGE_STATION_PITCH_VARIATION_V2))
            {
                this.currentState = State.Completed;
            }
        }

        switch (this.currentState)
        {
            case Balancing:
                // if the pitch diff over the past 0.1 seconds is greater than the acceptable diff
                if (Math.abs(this.angleRateAverage) <= TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF_V2)
                {
                    boolean reverseBalancing = this.orientation == Orientation.Backwards || this.orientation == Orientation.Right;

                    // if negative pitch, move forward
                    if (this.angle < 0)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, (reverseBalancing ? -1.0 : 1.0) * TuningConstants.CHARGE_STATION_BALANCING_SPEED_V2);
                    }
                    else // if (this.pitch > 0)
                    {
                        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, (reverseBalancing ? 1.0 : -1.0) * TuningConstants.CHARGE_STATION_BALANCING_SPEED_V2);
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