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

public class GoOverChargeStationTask extends ControlTaskBase
{
    private enum State
    {
        Starting,
        Climbing,
        Centered,
        Completed
    }

    private final double reverse;
    private final boolean backwardOrientation;

    private PigeonManager imuManager;
    private ITimer timer;

    private FloatingAverageCalculator pitchRateAverageCalculator; 
    private double pitchRateAverage;

    private State currentState;
    private double pitch;
    private double climbingExceededTransitionTime;

    public GoOverChargeStationTask(boolean reverse)
    {
        this(reverse, false);
    }

    public GoOverChargeStationTask(boolean reverse, boolean backwardOrientation)
    {
        this.backwardOrientation = backwardOrientation;
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
        this.pitchRateAverage = 0.0;
        this.pitchRateAverageCalculator = new FloatingAverageCalculator(this.timer, 0.25, 50);
        this.pitch = this.imuManager.getPitch();

        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, true);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, this.backwardOrientation ? 180.0 : 0.0);
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
            if (Math.abs(this.pitch) >= (TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_PITCH - TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_ACCEPTABLE_VARIATION))
            {
                this.currentState = State.Centered;
            }
                    
                
            
        }
        else if (this.currentState == State.Centered)
        {
            if (Math.abs(this.pitch) <= TuningConstants.CHARGE_STATION_PITCH_VARIATION)
            {
                this.currentState = State.Completed;
            }
        }

        switch (this.currentState)
        {
            case Centered:
            
                //uses climbing speed
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_CLIMBING_SPEED_V2 * this.reverse);
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