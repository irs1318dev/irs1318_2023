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

        if (this.currentState == State.Approaching)
        {
            if (this.pitch <= -TuningConstants.CHARGE_STATION_2_MOUNTING_TRANSITION_PITCH)
            {
                this.currentState = State.Mounting;
            }
        }
        else if (this.currentState == State.Mounting)
        {
            if (this.gyro <= TuningConstants.CHARGE_STATION_2_CLIMBING_TRANSITION_GYRO)
            {
                this.currentState = State.Climbing;
            }
        }
        else if (this.currentState == State.Climbing)
        {
            if (this.gyro >= TuningConstants.CHARGE_STATION_2_BRAKE_GYRO ||
                this.pitch >= TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF)
            {
                this.currentState = State.Balancing;

                // initial braking
                if (this.lastBrake == 0.0)
                {
                    this.lastBrake = currTime;
                }
            }
        }
        else if (this.currentState == State.Balancing)
        {
            if (Math.abs(this.pitch) <= TuningConstants.CHARGE_STATION_PITCH_VARIATION &&
                Math.abs(this.gyro) <= TuningConstants.CHARGE_STATION_2_COMPLETED_GYRO)
            {
                this.currentState = State.Completed;
            }
        }

        System.out.println(this.currentState.toString());
        switch (this.currentState)
        {
            case Balancing:
                if (this.lastBrake + TuningConstants.CHARGE_STATION_2_MIN_BRAKE_TIME > currTime)
                {
                    // brake if withing brake time
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                }
                else if (this.pitch < -TuningConstants.CHARGE_STATION_PITCH_VARIATION)
                {
                    // if negative pitch, move forward
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_2_BALANCING_SPEED);
                }
                else if (this.pitch > TuningConstants.CHARGE_STATION_PITCH_VARIATION)
                {
                    // if positive pitch, move backward
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -TuningConstants.CHARGE_STATION_2_BALANCING_SPEED);
                }
                else
                {
                    // if pitch diff is within acceptable range, then pause.
                    this.lastBrake = currTime;
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
                }

                break;

            case Approaching:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_2_APPROACH_SPEED * this.reverse);
                break;

            case Mounting:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_2_MOUNT_SPEED * this.reverse);
                break;

            case Climbing:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_2_CLIMBING_SPEED * this.reverse);
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