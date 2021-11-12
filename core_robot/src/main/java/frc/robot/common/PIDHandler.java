package frc.robot.common;

import frc.robot.common.robotprovider.ITimer;

/**
 * This class is a PID handler with a feed-forward handler and a complementary filter.
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
    private static final double MinTimeStep = 0.01;
    private final Double minOutput;
    private final Double maxOutput;

    private final ComplementaryFilter errorFilter;
    private final ComplementaryFilter outputFilter;

    // instance PIDFS constants
    private double kp;        // proportion for proportional
    private double ki;        // proportion for integral
    private double kd;        // proportion for derivative
    private double kf;        // proportion for feed-forward
    private double ks;        // multiplicand for adjusting scale of setpoint to match scale of measured value

    // instance variables
    private double integral = 0.0;          // integral of error data in memory
    private double prevMeasuredValue = 0.0; // the previous measured value
    private double prevTime = 0.0;          // the timestamp of our previous measurement
    private double prevError = 0.0;         // the error during our previous measurement
    private double output = 0.0;            // the output we wish to set after our calculation

    // other vars
    private final ITimer timer;

    /**
     * This constructor initializes the object and sets constants to affect gain.
     * This defaults to not utilizing a complementary filter to slow ramp-up/ramp-down.
     * 
     * @param kp scalar for proportional component
     * @param ki scalar for integral component
     * @param kd scalar for derivative component
     * @param kf scalar for feed-forward control
     * @param ks scalar for adjusting scale difference between measured value and setpoint value
     * @param minOutput indicates the minimum output value acceptable, or null
     * @param maxOutput indicates the maximum output value acceptable, or null
     * @param timer to track how much time has passed
     */
    public PIDHandler(
        double kp,
        double ki,
        double kd,
        double kf,
        double ks,
        Double minOutput,
        Double maxOutput,
        ITimer timer)
    {
        this(kp, ki, kd, kf, ks, 0.0, 1.0, 0.0, 1.0, minOutput, maxOutput, timer);
    }

    /**
     * This constructor initializes the object and sets constants to affect gain.
     * This utilizes a complementary filter to slow ramp-up/ramp-down.
     * 
     * @param kp scalar for proportional component
     * @param ki scalar for integral component
     * @param kd scalar for derivative component
     * @param kf scalar for feed-forward control
     * @param ks scalar for adjusting scale difference between measured value and setpoint value
     * @param kO scalar for output complementary filter multiplier
     * @param kN scalar for output complementary filter multiplier
     * @param kEO scalar for error complementary filter multiplier
     * @param kEN scalar for error complementary filter multiplier
     * @param minOutput indicates the minimum output value acceptable, or null
     * @param maxOutput indicates the maximum output value acceptable, or null
     * @param timer to track how much time has passed
     */
    public PIDHandler(
        double kp,
        double ki,
        double kd,
        double kf,
        double ks,
        double kO,
        double kN,
        double kEO,
        double kEN,
        Double minOutput,
        Double maxOutput,
        ITimer timer)
    {
        this.ki = ki;
        this.kd = kd;
        this.kp = kp;
        this.kf = kf;
        this.ks = ks;

        this.errorFilter = new ComplementaryFilter(kEO, kEN);
        this.outputFilter = new ComplementaryFilter(kO, kN);

        this.minOutput = minOutput;
        this.maxOutput = maxOutput;

        this.timer = timer;
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
    public double calculatePosition(double setpoint, double measuredValue)
    {
        // update dt
        double curTime = this.timer.get();
        double dt = curTime - this.prevTime;

        // To prevent division by zero and over-aggressive measurement, output updates at a max of 1kHz
        if (dt >= PIDHandler.MinTimeStep)
        {
            this.prevTime = curTime;

            // calculate error
            double rawError = this.ks * setpoint - measuredValue;
            this.errorFilter.update(rawError);
            double error = rawError; // this.errorFilter.getValue();

            // calculate integral, limiting it based on MaxOutput/MinOutput
            double potentialI = this.ki * (this.integral + error * dt);
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
                this.integral += error; // * this.dt;
            }

            // calculate derivative
            double derivative = (error - this.prevError); // / this.dt;

            // store error
            this.prevError = error;

            double result = this.kp * error +   // proportional
                this.ki * this.integral +       // integral
                this.kd * derivative +          // derivative
                this.kf * setpoint;             // feed-forward

            if (this.maxOutput != null && result > this.maxOutput)
            {
                result = this.maxOutput;
            }
            else if (this.minOutput != null && result < this.minOutput)
            {
                result = this.minOutput;
            }

            // apply complementary filter to slow ramp-up/ramp-down
            this.outputFilter.update(result);
            this.output = result; // this.outputFilter.getValue();
            this.prevMeasuredValue = measuredValue;
        }
        else if (dt < 0.0)
        {
            this.prevTime = curTime;
        }

        return this.output;
    }

    /**
     * Calculate the desired output value based on the setpoint and measured value.
     * measuredValue should be in the same unit as the setpoint, typically a rate of change of something over time.
     * This method should be called in a loop and fed feedback data and setpoint changes
     * 
     * @param setpoint describes the goal velocity value
     * @param measuredValue describes the measured value
     * 
     * @return output value to be used
     */
    public double calculateVelocity(double setpoint, double measuredValue)
    {
        // update dt
        double curTime = this.timer.get();
        double dt = curTime - this.prevTime;

        // To prevent division by zero and over-aggressive measurement, output updates at a max of 100 Hz
        if (dt >= PIDHandler.MinTimeStep)
        {
            this.prevTime = curTime;

            double rawError = this.ks * setpoint - measuredValue;
            this.errorFilter.update(rawError);
            double error = rawError; // this.errorFilter.getValue();

            // calculate integral, limiting it based on MaxOutput/MinOutput
            double potentialI = this.ki * (this.integral + error * dt);
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
                this.integral += error * dt;
            }

            // calculate derivative
            double derivative = (error - this.prevError) / dt;

            // store error
            this.prevError = error;

            double result = this.kp * error +   // proportional
                this.ki * this.integral +       // integral
                this.kd * derivative +          // derivative
                this.kf * setpoint;             // feed-forward

            if (this.maxOutput != null && result > this.maxOutput)
            {
                result = this.maxOutput;
            }
            else if (this.minOutput != null && result < this.minOutput)
            {
                result = this.minOutput;
            }

            // apply complementary filter to slow ramp-up/ramp-down
            this.outputFilter.update(result);
            this.output = result; // this.outputFilter.getValue();
            this.prevMeasuredValue = measuredValue;
        }
        else if (dt < 0.0)
        {
            this.prevTime = curTime;
        }

        return this.output;
    }

    /**
     * Calculate the desired output value based on the history, setpoint, and measured value.
     * measuredValue should be in a different unit than the setpoint, where the setpoint is the rate of change
     * of something over time, whereas the measured value is of the current "something" that is changing.
     * This method should be called in a loop and fed feedback data and setpoint changes
     * 
     * @param setpoint describes the goal velocity value
     * @param measuredValue describes the measured value, where the measured value is the ticks on the encoder
     * 
     * @return output value to be used
     */
    public double calculateVelocityByTicks(double setpoint, double measuredValue)
    {
        // update dt
        double curTime = this.timer.get();
        double dt = curTime - this.prevTime;

        // To prevent division by zero and over-aggressive measurement, output updates at a max of 100 Hz
        if (dt >= PIDHandler.MinTimeStep)
        {
            this.prevTime = curTime;

            // calculate change in ticks since our last measurement
            double deltaX = measuredValue - this.prevMeasuredValue;
            double timeRatio = 0.02 / dt;

            this.errorFilter.update(this.ks * setpoint - deltaX * timeRatio);
            double error = this.errorFilter.getValue();

            // calculate integral, limiting it based on MaxOutput/MinOutput
            double potentialI = this.ki * (this.integral + error * dt);
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
                this.integral += error * dt;
            }

            // calculate derivative
            double derivative = (error - this.prevError) / dt;

            // store error
            this.prevError = error;

            double result = this.kp * error +   // proportional
                this.ki * this.integral +       // integral
                this.kd * derivative +          // derivative
                this.kf * setpoint;             // feed-forward

            if (this.maxOutput != null && result > this.maxOutput)
            {
                result = this.maxOutput;
            }
            else if (this.minOutput != null && result < this.minOutput)
            {
                result = this.minOutput;
            }

            // apply complementary filter to slow ramp-up/ramp-down
            this.outputFilter.update(result);
            this.output = this.outputFilter.getValue();
            this.prevMeasuredValue = measuredValue;
        }
        else if (dt < 0.0)
        {
            this.prevTime = curTime;
        }

        return this.output;
    }

    public double getCurrentOutput()
    {
        return this.output;
    }

    public double getError()
    {
        return this.prevError;
    }

    public void setKp(double kp)
    {
        this.kp = kp;
    }

    public void setKi(double ki)
    {
        this.ki = ki;
    }

    public void setKd(double kd)
    {
        this.kd = kd;
    }

    public void setKf(double kf)
    {
        this.kf = kf;
    }

    public void reset()
    {
        this.prevError = 0.0;
        this.prevMeasuredValue = 0.0;
        this.prevTime = this.timer.get();
        this.integral = 0.0;

        this.output = 0.0;

        this.errorFilter.reset();
        this.outputFilter.reset();
    }
}
