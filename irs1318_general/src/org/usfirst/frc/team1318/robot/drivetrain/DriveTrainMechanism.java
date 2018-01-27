package org.usfirst.frc.team1318.robot.drivetrain;

import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.Helpers;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.IMechanism;
import org.usfirst.frc.team1318.robot.common.PIDHandler;
import org.usfirst.frc.team1318.robot.common.wpilib.ITalonSRX;
import org.usfirst.frc.team1318.robot.common.wpilib.ITimer;
import org.usfirst.frc.team1318.robot.common.wpilib.IWpilibProvider;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXControlMode;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXFeedbackDevice;
import org.usfirst.frc.team1318.robot.common.wpilib.TalonSRXNeutralMode;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.driver.common.Driver;

import com.google.inject.Inject;

/**
 * Drivetrain mechanism.
 * The mechanism defines the logic that controls a mechanism given inputs and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs.
 * 
 */
@Singleton
public class DriveTrainMechanism implements IMechanism
{
    private static final String LogName = "dt";

    private static final int pidSlotId = 0;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private final IDashboardLogger logger;
    private final ITimer timer;

    private final ITalonSRX leftMotor;
    private final ITalonSRX rightMotor;

    private Driver driver;

    private PIDHandler leftPID;
    private PIDHandler rightPID;

    private boolean usePID;
    private boolean usePositionalMode;

    private double leftVelocity;
    private double leftError;
    private int leftPosition;
    private double rightVelocity;
    private double rightError;
    private int rightPosition;

    /**
     * Initializes a new DriveTrainMechanism
     * @param logger to use
     * @param provider for obtaining electronics objects
     * @param timer to use
     */
    @Inject
    public DriveTrainMechanism(
        IDashboardLogger logger,
        IWpilibProvider provider,
        ITimer timer)
    {
        this.logger = logger;
        this.timer = timer;

        this.leftMotor = provider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CHANNEL);
        this.leftMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.leftMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_LEFT_INVERT_OUTPUT);
        this.leftMotor.setInvertSensor(HardwareConstants.DRIVETRAIN_LEFT_INVERT_SENSOR);
        this.leftMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.leftMotor.setPIDF(
            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KP,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KI,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KD,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF,
            DriveTrainMechanism.pidSlotId);

        ITalonSRX leftFollowerMotor = provider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER_CHANNEL);
        leftFollowerMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        leftFollowerMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_LEFT_INVERT_OUTPUT);
        leftFollowerMotor.setControlMode(TalonSRXControlMode.Follower);
        leftFollowerMotor.set(ElectronicsConstants.DRIVETRAIN_LEFT_MOTOR_CHANNEL);

        this.rightMotor = provider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CHANNEL);
        this.rightMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        this.rightMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_OUTPUT);
        this.rightMotor.setInvertSensor(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_SENSOR);
        this.rightMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.rightMotor.setPIDF(
            TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KP,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KI,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KD,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KF,
            DriveTrainMechanism.pidSlotId);

        ITalonSRX rightFollowerMotor = provider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER_CHANNEL);
        rightFollowerMotor.setControlMode(TalonSRXControlMode.Follower);
        rightFollowerMotor.setNeutralMode(TalonSRXNeutralMode.Brake);
        rightFollowerMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_OUTPUT);
        rightFollowerMotor.set(ElectronicsConstants.DRIVETRAIN_RIGHT_MOTOR_CHANNEL);

        this.leftPID = null;
        this.rightPID = null;

        this.usePID = TuningConstants.DRIVETRAIN_USE_PID;
        this.usePositionalMode = false;

        this.leftVelocity = 0.0;
        this.leftError = 0.0;
        this.leftPosition = 0;
        this.rightVelocity = 0.0;
        this.rightError = 0.0;
        this.rightPosition = 0;
    }

    /**
     * get the velocity from the left encoder
     * @return a value indicating the velocity
     */
    public double getLeftVelocity()
    {
        return this.leftVelocity;
    }

    /**
     * get the velocity from the right encoder
     * @return a value indicating the velocity
     */
    public double getRightVelocity()
    {
        return this.rightVelocity;
    }

    /**
     * get the distance from the left encoder
     * @return a value indicating the distance
     */
    public double getLeftError()
    {
        return this.leftError;
    }

    /**
     * get the distance from the right encoder
     * @return a value indicating the distance
     */
    public double getRightError()
    {
        return this.rightError;
    }

    /**
     * get the ticks from the left encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getLeftPosition()
    {
        return this.leftPosition;
    }

    /**
     * get the ticks from the right encoder
     * @return a value indicating the number of ticks we are at
     */
    public int getRightTicks()
    {
        return this.rightPosition;
    }

    /**
     * set the driver that the mechanism should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;

        // switch to default velocity PID mode whenever we switch drivers (defense-in-depth)
        if (!this.usePID || this.usePositionalMode)
        {
            this.usePID = TuningConstants.DRIVETRAIN_USE_PID;
            this.usePositionalMode = false;
        }

        this.setControlMode();
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.leftVelocity = this.leftMotor.getVelocity();
        this.leftError = this.leftMotor.getError();
        this.leftPosition = this.leftMotor.getPosition();
        this.rightVelocity = this.rightMotor.getVelocity();
        this.rightError = this.rightMotor.getError();
        this.rightPosition = this.rightMotor.getPosition();

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftVelocity", this.leftVelocity);
        this.logger.logNumber(DriveTrainMechanism.LogName, "leftError", this.leftError);
        this.logger.logNumber(DriveTrainMechanism.LogName, "leftTicks", this.leftPosition);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightVelocity", this.rightVelocity);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightError", this.rightError);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightTicks", this.rightPosition);
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant mechanism
     */
    @Override
    public void update()
    {
        if (this.driver.getDigital(Operation.DriveTrainEnablePID))
        {
            this.usePID = true;
            this.setControlMode();
        }
        else if (this.driver.getDigital(Operation.DriveTrainDisablePID))
        {
            this.usePID = false;
            this.setControlMode();
        }

        // check our desired PID mode (needed for positional mode or break mode)
        boolean newUsePositionalMode = this.driver.getDigital(Operation.DriveTrainUsePositionalMode);
        if (newUsePositionalMode != this.usePositionalMode)
        {
            this.usePositionalMode = newUsePositionalMode;

            // re-create PID handler
            this.setControlMode();
        }

        // calculate desired power setting for the current mode
        Setpoint setpoint;
        if (!this.usePositionalMode)
        {
            setpoint = this.calculateVelocityModeSetpoint();
        }
        else
        {
            setpoint = this.calculatePositionModeSetpoint();
        }

        double leftSetpoint = setpoint.getLeft();
        double rightSetpoint = setpoint.getRight();

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftVelocityGoal", leftSetpoint);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightVelocityGoal", rightSetpoint);

        // apply the setpoints to the motors
        this.leftMotor.set(leftSetpoint);
        this.rightMotor.set(rightSetpoint);
    }

    /**
     * stop the relevant mechanism
     */
    @Override
    public void stop()
    {
        this.leftMotor.stop();
        this.rightMotor.stop();

        this.leftMotor.reset();
        this.rightMotor.reset();

        if (this.leftPID != null)
        {
            this.leftPID.reset();
        }

        if (this.rightPID != null)
        {
            this.rightPID.reset();
        }

        this.leftVelocity = 0.0;
        this.leftError = 0.0;
        this.leftPosition = 0;
        this.rightVelocity = 0.0;
        this.rightError = 0.0;
        this.rightPosition = 0;
    }

    /**
     * create a PIDHandler based on our current settings
     */
    private void setControlMode()
    {
        TalonSRXControlMode mode = TalonSRXControlMode.PercentOutput;
        if (this.usePID)
        {
            if (this.usePositionalMode)
            {
                this.leftPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KP,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KI,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KD,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KF,
                    1.0,
                    -TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    this.timer);
                this.rightPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KP,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KI,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KD,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KF,
                    1.0,
                    -TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    this.timer);
            }
            else
            {
                this.leftPID = null;
                this.rightPID = null;
            }

            mode = TalonSRXControlMode.Velocity;
            this.leftMotor.setSelectedSlot(DriveTrainMechanism.pidSlotId);
            this.rightMotor.setSelectedSlot(DriveTrainMechanism.pidSlotId);
        }

        this.leftMotor.setControlMode(mode);
        this.rightMotor.setControlMode(mode);
    }

    /**
     * Calculate the power setting to use based on the inputs when in velocity mode
     * @return power settings for left and right motor
     */
    private Setpoint calculateVelocityModeSetpoint()
    {
        // velocity goals represent the desired percentage of the max velocity
        double leftVelocityGoal = 0.0;
        double rightVelocityGoal = 0.0;

        // get a value indicating that we should be in simple mode...
        boolean simpleDriveModeEnabled = this.driver.getDigital(Operation.DriveTrainSimpleMode);

        // get the X and Y values from the operator.  We expect these to be between -1.0 and 1.0,
        // with this value representing the forward velocity percentage and right turn percentage (of max speed)
        double turnAmount = this.driver.getAnalog(Operation.DriveTrainTurn);
        double forwardVelocity = this.driver.getAnalog(Operation.DriveTrainMoveForward);

        // Negate the x and y if DriveTrainSwapFrontOrientation is true
        if (this.driver.getDigital(Operation.DriveTrainSwapFrontOrientation))
        {
            turnAmount *= -1.0;
            forwardVelocity *= -1.0;
        }

        // adjust the intensity of the input
        if (simpleDriveModeEnabled)
        {
            if (Math.abs(forwardVelocity) < Math.abs(turnAmount))
            {
                // in-place turn
                leftVelocityGoal = turnAmount;
                rightVelocityGoal = -turnAmount;
            }
            else
            {
                // forward/backward
                leftVelocityGoal = forwardVelocity;
                rightVelocityGoal = forwardVelocity;
            }
        }
        else
        {
            leftVelocityGoal = (TuningConstants.DRIVETRAIN_K1 * forwardVelocity) + (TuningConstants.DRIVETRAIN_K2 * turnAmount);
            rightVelocityGoal = (TuningConstants.DRIVETRAIN_K1 * forwardVelocity) + (-TuningConstants.DRIVETRAIN_K2 * turnAmount);
        }

        // decrease the desired velocity based on the configured max power level
        leftVelocityGoal = leftVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;
        rightVelocityGoal = rightVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;

        // ensure that we don't give values outside the appropriate range
        double left = this.applyPowerLevelRange(leftVelocityGoal);
        double right = this.applyPowerLevelRange(rightVelocityGoal);

        this.assertPowerLevelRange(left, "left");
        this.assertPowerLevelRange(right, "right");

        // if we are using PID, then we base the setpoint on the max velocity
        if (this.usePID)
        {
            left *= TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS;
            right *= TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KS;
        }

        return new Setpoint(left, right);
    }

    /**
     * Calculate the power setting to use based on the inputs when in position mode
     * @return power settings for left and right motor
     */
    private Setpoint calculatePositionModeSetpoint()
    {
        // get the desired left and right values from the driver.
        double leftPositionGoal = this.driver.getAnalog(Operation.DriveTrainLeftPosition);
        double rightPositionGoal = this.driver.getAnalog(Operation.DriveTrainRightPosition);

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftPositionGoal", leftPositionGoal);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightPositionGoal", rightPositionGoal);

        double left;
        double right;
        if (this.usePID)
        {
            // use positional PID to get the relevant value
            left = this.leftPID.calculatePosition(leftPositionGoal, this.leftPosition);
            right = this.rightPID.calculatePosition(rightPositionGoal, this.rightPosition);
        }
        else
        {
            // calculate a desired power level
            left = leftPositionGoal - this.leftPosition;
            right = rightPositionGoal - this.rightPosition;
            if (Math.abs(left) < 0.1)
            {
                left = 0.0;
            }

            if (Math.abs(right) < 0.1)
            {
                right = 0.0;
            }

            left *= TuningConstants.DRIVETRAIN_LEFT_POSITIONAL_NON_PID_MULTIPLICAND;
            right *= TuningConstants.DRIVETRAIN_RIGHT_POSITIONAL_NON_PID_MULTIPLICAND;

            // ensure that we are within our power level range, and then scale it down
            left = this.applyPowerLevelRange(left) * TuningConstants.DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
            right = this.applyPowerLevelRange(right) * TuningConstants.DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
        }

        this.assertPowerLevelRange(left, "left velocity (goal)");
        this.assertPowerLevelRange(right, "right velocity (goal)");

        if (this.usePID)
        {
            left *= TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS;
            right *= TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KS;
        }

        return new Setpoint(left, right);
    }

    /**
     * Assert that the power level is within the required range
     * @param powerLevel to verify
     * @param side indicator for the exception message if incorrect
     */
    private void assertPowerLevelRange(double powerLevel, String side)
    {
        if (powerLevel < DriveTrainMechanism.POWERLEVEL_MIN)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too low!");
            }

            return;
        }

        if (powerLevel > DriveTrainMechanism.POWERLEVEL_MAX)
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException(side + " power level too high!");
            }

            return;
        }
    }

    /**
     * Reset the power level to be within the required range
     * @param powerLevel to reset
     * @return power level
     */
    private double applyPowerLevelRange(double powerLevel)
    {
        return Helpers.EnforceRange(powerLevel, DriveTrainMechanism.POWERLEVEL_MIN, DriveTrainMechanism.POWERLEVEL_MAX);
    }

    /**
     * Simple holder of setpoint information for the left and right sides
     */
    private class Setpoint
    {
        private double left;
        private double right;

        /**
         * Initializes a new Setpoint
         * @param left value to apply
         * @param right value to apply
         */
        public Setpoint(double left, double right)
        {
            this.left = left;
            this.right = right;
        }

        /**
         * gets the left setpoint
         * @return left setpoint value
         */
        public double getLeft()
        {
            return this.left;
        }

        /**
         * gets the right setpoint
         * @return right setpoint value
         */
        public double getRight()
        {
            return this.right;
        }
    }
}
