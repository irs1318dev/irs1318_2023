package org.usfirst.frc.team1318.robot.Common;

import edu.wpi.first.wpilibj.Timer;

/**
 * This class is a PID handler with a feed-forward handler.
 * 
 * To use PID control:
 *      set the kp/ki/kd/kf tuning values
 *      calculate output based on the setpoint and measured value regularly
 * 
 * for reference:
 *      http://en.wikipedia.org/wiki/PID_controller
 *      http://en.wikipedia.org/wiki/Feed_forward_(control)
 * 
 * @author Will (adapted from old code)
 */

public class PIDHandler
{
    // constants
    private static final double MinTimeStep = .001;
    private final Double minOutput;
    private final Double maxOutput;

    // instance constants
    private final double kp;        // proportion for proportional
    private final double ki;        // proportion for integral
    private final double kd;        // proportion for derivative
    private final double kf;        // proportion for feed-forward

    // instance variables
    private double setpoint = 0.0;      // the input, desired value for
    private double measuredValue = 0.0; // the measured value for PID 
    private double integral = 0.0;      // integral of error data in memory
    private double derivative = 0.0;    // approximate slope of input.. units in / seconds
    private double dt = .001;           // amount of time we waited since our previous measurement
    private double prevTime = 0.0;      // the timestamp of our previous measurement 
    private double error = 0.0;         // the error (difference between setpoint and measured value)
    private double prevError = 0.0;     // the error during our previous measurement
    private double curTime = 0.0;       // the current timestamp
    private double output = 0.0;        // the output we wish to set after our calculation

    // other vars
    private Timer timer;

    /**
     * This constructor initializes the object and sets constants to affect gain
     * 
     * @param kp scalar for proportional component
     * @param ki scalar for integral component
     * @param kd scalar for derivative component
     * @param kf scalar for feed-forward control
     * @param minOutput indicates the minimum output value acceptable, or null
     * @param maxOutput indicates the maximum output value acceptable, or null
     */
    public PIDHandler(double kp, double ki, double kd, double kf, Double minOutput, Double maxOutput)
    {
        this.ki = ki;
        this.kd = kd;
        this.kp = kp;
        this.kf = kf;

        this.minOutput = minOutput;
        this.maxOutput = maxOutput;

        this.timer = new Timer();
        this.timer.start();
        this.prevTime = this.timer.get();
    }

    /**
     * Calculate the desired output value based on the history, setpoint, and measured value.
     * measuredValue should be in the same unit as the setpoint, basically a positive or negative percentage 
     * between -1 and 1.  This method should be called in a loop and fed feedback data and setpoint changes
     * 
     * @param setpoint describes the goal value
     * @param measuredValue describes the measured value
     * 
     * @return output value to be used
     */
    public double calculate(double setpoint, double measuredValue)
    {
        this.setpoint = setpoint;
        this.measuredValue = measuredValue;

        // update dt
        this.curTime = this.timer.get();
        this.dt = this.curTime - this.prevTime;

        // To prevent division by zero and over-aggressive measurement, output updates at a max of 1kHz
        if (this.dt >= PIDHandler.MinTimeStep)
        {
            this.prevTime = this.curTime;

            // calculate error
            this.error = this.setpoint - this.measuredValue;

            // calculate integral, limiting it based on MaxOutput/MinOutput
            double potentialI = this.ki * (this.integral + this.error * this.dt);
            if (this.maxOutput != null && potentialI > this.maxOutput)
            {
                this.integral = this.maxOutput / this.ki;
            }
            else if (this.minOutput != null && potentialI < this.minOutput)
            {
                this.integral = this.minOutput / this.ki;
            }
            else
            {
                this.integral += this.error * this.dt;
            }

            // calculate derivative
            this.derivative = (this.error - this.prevError) / this.dt;

            // store error
            this.prevError = this.error;

            double result =
                this.kp * this.error +      // proportional
                this.ki * this.integral +   // integral
                    this.kd * this.derivative + // derivative
                    this.kf * this.setpoint;    // feed-forward

            if (this.maxOutput != null && result > this.maxOutput)
            {
                result = this.maxOutput;
            }
            else if (this.minOutput != null && result < this.minOutput)
            {
                result = this.minOutput;
            }

            this.output = result;
        }

        return this.output;
    }
}
