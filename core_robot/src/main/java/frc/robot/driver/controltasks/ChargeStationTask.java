//Jamie Hsieh, Calvin Rodrigue
//1.19.2023
//Assumes the robot is on the center platform, and adjusts position.
//Jamie's first robot code commit.

package frc.robot.driver.controltasks;

import com.google.inject.Injector;

import frc.robot.TuningConstants;
import frc.robot.TuningConstants.*;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.PigeonManager;

public class ChargeStationTask extends ControlTaskBase
{
    

    private PigeonManager imuManager;
    private DriveTrainMechanism driveTrain;
    private LoggingManager logger;
    private ITimer timer;

    private double pitch;
    private double sign;
    private double xPos;
    private double balancePos;
    private double distanceTraveled;
    private double timeSinceLastPitchLog;
    private double prevPitchLogTime;
    private double[] pitchLog = new double[25]; //logs every 0.02 secs, array is 0.5 secs total
    private enum State{
        Starting,
        Climbing,
        Balancing,
        Tipping,
        Completed
    }
    private State currentState;
    

    public ChargeStationTask()
    {
        this.pitch = 0.0;
        this.sign = 1.0;
        this.xPos = 1.0;
        this.balancePos = 0.0;
        this.distanceTraveled = 0.0;
        this.timeSinceLastPitchLog = 0.0;
        this.prevPitchLogTime = 0.0;
        
        
    }

    private double findDiff() //finds the absolute value diff of pitch values of beginning and end of pitchLog[]
    {
        return Math.abs((this.pitchLog[this.pitchLog.length - 1]) - this.pitchLog[0]);
    }

    /**
     * Begin the current task.
     */
    @Override
    public void begin()
    {
        currentState = State.Starting;

        this.imuManager = this.getInjector().getInstance(PigeonManager.class);
        this.driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);
        this.logger = this.getInjector().getInstance(LoggingManager.class);
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.pitch = this.imuManager.getPitch();
        this.prevPitchLogTime = this.timer.get();
        this.timeSinceLastPitchLog = this.timer.get() - this.prevPitchLogTime;

        //at the beginning of task set all array values to be current pitch
        for (int i = 0; i < this.pitchLog.length; i++)
            {
                this.pitchLog[i] = this.pitch;
            }
        
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, true);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, 0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0);

        
    }

    /**
     * Run an iteration of the current task.
     */
    @Override
    public void update()
    {

        

        this.timeSinceLastPitchLog = this.timer.get() - this.prevPitchLogTime; //current time - previous pitch log time


        //previous pitch log movement
        if (this.timeSinceLastPitchLog >= 0.02)
        {
            // if previous pitch log time was 0.02 secs ago then set pitch log time as current time, and log
            this.prevPitchLogTime = this.timer.get(); 

            // constantly update array for past 0.5 seconds
            for (int i = 1; i < this.pitchLog.length; i++)
            {
                this.pitchLog[i-1] = this.pitchLog[i];
            }

            // put current pitch in final space
            this.pitchLog[this.pitchLog.length - 1] = this.pitch;
        }

        //if the pitch is greater than 15 degrees (both wheels on center platform) move forward
        this.pitch = imuManager.getPitch();

        if (this.currentState == State.Starting)
        {
                //if front wheel is on first part, set switch to climbing
                if (this.pitch >= TuningConstants.CHARGE_STATION_START_TRANSITION_PITCH)
                {
                    this.currentState = State.Climbing;

                }
        }
        else if (this.currentState == State.Climbing)
        {
                //if pitch is larger than 15-ish, set speed to balancing speed
                if (this.pitch >= (TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_PITCH - TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_ACCEPTABLE_VARIATION))
                {
                    this.currentState = State.Balancing;

                }
        }
        else if (this.currentState == State.Balancing)
        {
            //if pitch is larger than 15-ish, set speed to balancing speed
            if (this.pitch >= (TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_PITCH - TuningConstants.CHARGE_STATION_CLIMBING_TRANSITION_ACCEPTABLE_VARIATION))
            {
                this.currentState = State.Balancing;
            }
        }

        switch (this.currentState)
        {
            case Balancing:
                this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_BALANCING_SPEED);

                //if the pitch diff of 0.5 seconds is greater than the acceptable diff
                if (TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF >= (findDiff()))
                {
                    if (this.pitch < 0) //if negative pitch, move forward
                    {
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, TuningConstants.CHARGE_STATION_BALANCING_SPEED);
                    }

                    if (this.pitch > 0) //if positive pitch, move backward
                    {
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, - TuningConstants.CHARGE_STATION_BALANCING_SPEED);
                    }
                }

                //wait to see if leveled
                else
                {
                    this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0);
                }

                break;

            case Climbing:
                break;

            case Starting:
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
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0);
    }


    /**
     * Checks whether this task has completed, or whether it should continue being processed.
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return this.currentState == State.Completed;
        //if the diff of past 0.5 seconds is within acceptable pitch diff, 
        //and the current pitch is within acceptable leveled variation, end.
        if ((TuningConstants.CHARGE_STATION_ACCEPTABLE_PITCH_DIFF >= findDiff()) && 
        (Math.abs(this.pitch) <= TuningConstants.CHARGE_STATION_PITCH_VARIATION)) 
        {
            return true;
        }

        return false;
    }

    
}