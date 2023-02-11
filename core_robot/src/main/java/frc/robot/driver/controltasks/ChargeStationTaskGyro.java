//Jamie Hsieh, Calvin Rodrigue
//1.19.2023
//Assumes the robot is on the center platform, and adjusts position.
//Jamie's first robot code commit.

package frc.robot.driver.controltasks;

import frc.robot.TuningConstants;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.*;
import frc.robot.mechanisms.*;

public class ChargeStationTaskGyro extends ControlTaskBase
{
    private enum State
    {
        Approaching,
        Mounting,
        Climbing,
        Balancing,
        Completed
    }

    private State currentState;

    private PigeonManager imuManager;
    private ITimer timer;

    private double pitch;
    private double gyro;
    private double lastBrake;

    //private final double[] pitchLog;
    //private double prevPitchLogTime;

    private double reverse = 1.0;

    public ChargeStationTaskGyro(boolean reverse)
    {
        this.reverse = 1.0;
        if (reverse)
        {
            this.reverse = -1.0;
        }

        this.currentState = State.Mounting;
        this.pitch = 0.0;
        this.gyro = 0.0;
        this.lastBrake = 0.0;

        //this.pitchLog = new double[25]; // logs every 0.02 secs, array is 0.5 secs total
        //this.prevPitchLogTime = 0.0;
    }

    /**
     * Begin the current task.
     */
    @Override
    public void begin()
    {
        this.currentState = State.Approaching;

        this.imuManager = this.getInjector().getInstance(PigeonManager.class);
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.pitch = this.imuManager.getPitch();
        this.gyro = this.imuManager.getPitchRate();

        // at the beginning of task set all array values to be current pitch
        // for (int i = 0; i < this.pitchLog.length; i++)
        // {
        //     this.pitchLog[i] = this.pitch;
        // }

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
        this.gyro = this.imuManager.getPitchRate();

        if (this.currentState == State.Approaching) {
            if (this.gyro >= TuningConstants.MOUNTING_TRANSITION_GYRO) {
                this.currentState = State.Mounting;
            }
        }
        else if (this.currentState == State.Mounting) {
            if (this.gyro <= TuningConstants.CLIMBING_TRANSITION_GYRO) {
                this.currentState = State.Climbing;
            }
        }
        else if (this.currentState == State.Climbing) {
            if (this.gyro >= TuningConstants.BRAKE_GYRO || this.pitch >= TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF) {
                this.currentState = State.Balancing;
                //initial braking
                if (this.lastBrake == 0.0) {
                    this.lastBrake = currTime;
                }
            }
        }
        else if (this.currentState == State.Balancing)
        {
            if (Math.abs(this.pitch) <= TuningConstants.CHARGE_STATION_PITCH_VARIATION &&
                    Math.abs(this.gyro) <= TuningConstants.COMPLETED_GYRO) {
                this.currentState = State.Completed;
            }
        }
        

        switch (this.currentState)
        {
            case Balancing:
                //brake if withing brake time
                if (this.lastBrake + TuningConstants.MIN_BRAKE_TIME > currTime) {
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                }
                
                // if negative pitch, move forward
                else if (this.pitch < -TuningConstants.CHARGE_STATION_PITCH_VARIATION)
                {
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_BALANCING_SPEED);
                }
                else if (this.pitch > TuningConstants.CHARGE_STATION_PITCH_VARIATION)
                {
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -TuningConstants.CHARGE_STATION_BALANCING_SPEED);
                }
                else // if pitch diff is within acceptable range, then pause.
                {
                    this.lastBrake = currTime;
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                }

                break;

            case Approaching:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.APPROACH_SPEED * this.reverse);
                break;

            case Mounting:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.MOUNT_SPEED * this.reverse);
                break;
            case Climbing:
            this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.MOUNT_SPEED * this.reverse);
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