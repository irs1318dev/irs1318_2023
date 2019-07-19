package frc.robot.mechanisms;

import javax.inject.Singleton;

import frc.robot.ElectronicsConstants;
import frc.robot.HardwareConstants;
import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.common.IMechanism;
import frc.robot.common.PIDHandler;
import frc.robot.common.robotprovider.IDashboardLogger;
import frc.robot.common.robotprovider.IRobotProvider;
import frc.robot.common.robotprovider.ITalonSRX;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.IVictorSPX;
import frc.robot.common.robotprovider.TalonSRXControlMode;
import frc.robot.common.robotprovider.TalonSRXFeedbackDevice;
import frc.robot.common.robotprovider.MotorNeutralMode;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.Driver;

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
    private static final int FRAME_PERIOD_MS = 5;

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
    private boolean useSimplePathMode;
    private boolean usePathMode;
    private boolean usePositionalMode;
    private boolean useBrakeMode;

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
        IRobotProvider provider,
        ITimer timer)
    {
        this.logger = logger;
        this.timer = timer;

        this.leftMotor = provider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_MASTER_CAN_ID);
        this.leftMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.leftMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_LEFT_MASTER_INVERT_OUTPUT);
        this.leftMotor.setInvertSensor(HardwareConstants.DRIVETRAIN_LEFT_INVERT_SENSOR);
        this.leftMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.leftMotor.setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
        this.leftMotor.setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
        this.leftMotor.configureVelocityMeasurements();
        this.leftMotor.setPIDF(
            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KP,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KI,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KD,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF,
            DriveTrainMechanism.pidSlotId);

        IVictorSPX leftFollowerMotor1 = provider.getVictorSPX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER1_CAN_ID);
        leftFollowerMotor1.setNeutralMode(MotorNeutralMode.Brake);
        leftFollowerMotor1.setInvertOutput(HardwareConstants.DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT);
        leftFollowerMotor1.follow(this.leftMotor);

        ITalonSRX leftFollowerMotor2 = provider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER2_CAN_ID);
        leftFollowerMotor2.setNeutralMode(MotorNeutralMode.Brake);
        leftFollowerMotor2.setInvertOutput(HardwareConstants.DRIVETRAIN_LEFT_FOLLOWER2_INVERT_OUTPUT);
        leftFollowerMotor2.follow(this.leftMotor);

        this.rightMotor = provider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_MASTER_CAN_ID);
        this.rightMotor.setNeutralMode(MotorNeutralMode.Brake);
        this.rightMotor.setInvertOutput(HardwareConstants.DRIVETRAIN_RIGHT_MASTER_INVERT_OUTPUT);
        this.rightMotor.setInvertSensor(HardwareConstants.DRIVETRAIN_RIGHT_INVERT_SENSOR);
        this.rightMotor.setSensorType(TalonSRXFeedbackDevice.QuadEncoder);
        this.rightMotor.setFeedbackFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
        this.rightMotor.setPIDFFramePeriod(DriveTrainMechanism.FRAME_PERIOD_MS);
        this.rightMotor.configureVelocityMeasurements();
        this.rightMotor.setPIDF(
            TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KP,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KI,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KD,
            TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KF,
            DriveTrainMechanism.pidSlotId);

        IVictorSPX rightFollowerMotor1 = provider.getVictorSPX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER1_CAN_ID);
        rightFollowerMotor1.setNeutralMode(MotorNeutralMode.Brake);
        rightFollowerMotor1.setInvertOutput(HardwareConstants.DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT);
        rightFollowerMotor1.follow(this.rightMotor);

        ITalonSRX rightFollowerMotor2 = provider.getTalonSRX(ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER2_CAN_ID);
        rightFollowerMotor2.setNeutralMode(MotorNeutralMode.Brake);
        rightFollowerMotor2.setInvertOutput(HardwareConstants.DRIVETRAIN_RIGHT_FOLLOWER2_INVERT_OUTPUT);
        rightFollowerMotor2.follow(this.rightMotor);

        this.leftPID = null;
        this.rightPID = null;

        this.usePID = TuningConstants.DRIVETRAIN_USE_PID;
        this.useSimplePathMode = false;
        this.usePathMode = false;
        this.usePositionalMode = false;
        this.useBrakeMode = false;

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
    public int getRightPosition()
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
        if (!this.usePID || this.useSimplePathMode || this.usePathMode || this.usePositionalMode || this.useBrakeMode)
        {
            this.usePID = TuningConstants.DRIVETRAIN_USE_PID;
            this.useSimplePathMode = false;
            this.usePathMode = false;
            this.usePositionalMode = false;
            this.useBrakeMode = false;
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
        this.rightVelocity = this.rightMotor.getVelocity();

        this.leftPosition = this.leftMotor.getPosition();
        this.rightPosition = this.rightMotor.getPosition();

        this.leftError = this.leftMotor.getError();
        this.rightError = this.rightMotor.getError();

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
        if (this.driver.getDigital(DigitalOperation.DriveTrainEnablePID))
        {
            this.usePID = true;
            this.setControlMode();
        }
        else if (this.driver.getDigital(DigitalOperation.DriveTrainDisablePID))
        {
            this.usePID = false;
            this.setControlMode();
        }

        // check our desired PID mode (needed for positional mode or break mode)
        boolean newUseSimplePathMode = this.driver.getDigital(DigitalOperation.DriveTrainUseSimplePathMode);
        boolean newUsePathMode = this.driver.getDigital(DigitalOperation.DriveTrainUsePathMode);
        boolean newUsePositionalMode = this.driver.getDigital(DigitalOperation.DriveTrainUsePositionalMode);
        boolean newUseBrakeMode = this.driver.getDigital(DigitalOperation.DriveTrainUseBrakeMode);
        if (newUseSimplePathMode != this.useSimplePathMode ||
            newUsePathMode != this.usePathMode ||
            newUsePositionalMode != this.usePositionalMode ||
            newUseBrakeMode != this.useBrakeMode)
        {
            this.useSimplePathMode = newUseSimplePathMode;
            this.usePathMode = newUsePathMode;
            this.usePositionalMode = newUsePositionalMode;
            this.useBrakeMode = newUseBrakeMode;

            // re-create PID handler
            this.setControlMode();
        }

        // calculate desired setting for the current mode
        Setpoint setpoint;
        if (this.useSimplePathMode)
        {
            setpoint = this.calculateSimplePathModeSetpoint();
        }
        else if (this.usePathMode)
        {
            setpoint = this.calculatePathModeSetpoint();
        }
        else if (this.usePositionalMode)
        {
            setpoint = this.calculatePositionModeSetpoint();
        }
        else
        {
            setpoint = this.calculateVelocityModeSetpoint();
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
            if (this.usePathMode)
            {
                this.leftPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_PATH_PID_LEFT_KP,
                    TuningConstants.DRIVETRAIN_PATH_PID_LEFT_KI,
                    TuningConstants.DRIVETRAIN_PATH_PID_LEFT_KD,
                    TuningConstants.DRIVETRAIN_PATH_PID_LEFT_KF,
                    1.0,
                    -TuningConstants.DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    this.timer);
                this.rightPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_PATH_PID_RIGHT_KP,
                    TuningConstants.DRIVETRAIN_PATH_PID_RIGHT_KI,
                    TuningConstants.DRIVETRAIN_PATH_PID_RIGHT_KD,
                    TuningConstants.DRIVETRAIN_PATH_PID_RIGHT_KF,
                    1.0,
                    -TuningConstants.DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_PATH_MAX_POWER_LEVEL,
                    this.timer);
            }
            else if (this.usePositionalMode)
            {
                if (this.useBrakeMode)
                {
                    this.leftPID = new PIDHandler(
                        TuningConstants.DRIVETRAIN_BRAKE_PID_LEFT_KP,
                        TuningConstants.DRIVETRAIN_BRAKE_PID_LEFT_KI,
                        TuningConstants.DRIVETRAIN_BRAKE_PID_LEFT_KD,
                        TuningConstants.DRIVETRAIN_BRAKE_PID_LEFT_KF,
                        1.0,
                        -TuningConstants.DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        TuningConstants.DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        this.timer);
                    this.rightPID = new PIDHandler(
                        TuningConstants.DRIVETRAIN_BRAKE_PID_RIGHT_KP,
                        TuningConstants.DRIVETRAIN_BRAKE_PID_RIGHT_KI,
                        TuningConstants.DRIVETRAIN_BRAKE_PID_RIGHT_KD,
                        TuningConstants.DRIVETRAIN_BRAKE_PID_RIGHT_KF,
                        1.0,
                        -TuningConstants.DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        TuningConstants.DRIVETRAIN_BRAKE_MAX_POWER_LEVEL,
                        this.timer);
                }
                else
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
     * Calculate the setting to use based on the inputs when in velocity mode
     * @return settings for left and right motor
     */
    private Setpoint calculateVelocityModeSetpoint()
    {
        // velocity goals represent the desired percentage of the max velocity
        double leftVelocityGoal = 0.0;
        double rightVelocityGoal = 0.0;

        // get a value indicating that we should be in simple mode...
        boolean simpleDriveModeEnabled = this.driver.getDigital(DigitalOperation.DriveTrainSimpleMode);

        // get the X and Y values from the operator.  We expect these to be between -1.0 and 1.0,
        // with this value representing the forward velocity percentage and right turn percentage (of max speed)
        double turnAmount = this.driver.getAnalog(AnalogOperation.DriveTrainTurn);
        double forwardVelocity = this.driver.getAnalog(AnalogOperation.DriveTrainMoveForward);

        // Negate the x and y if DriveTrainSwapFrontOrientation is true
        if (this.driver.getDigital(DigitalOperation.DriveTrainSwapFrontOrientation))
        {
            turnAmount *= -1.0;
            forwardVelocity *= -1.0;
        }

        if ((!simpleDriveModeEnabled && TuningConstants.DRIVETRAIN_REGULAR_MODE_SQUARING)
            || (simpleDriveModeEnabled && TuningConstants.DRIVETRAIN_SIMPLE_MODE_SQUARING))
        {
            if (turnAmount >= 0)
            {
                turnAmount = turnAmount * turnAmount;
            }
            else
            {
                turnAmount = -1.0 * turnAmount * turnAmount;
            }

            if (forwardVelocity >= 0)
            {
                forwardVelocity = forwardVelocity * forwardVelocity;
            }
            else
            {
                forwardVelocity = -1.0 * forwardVelocity * forwardVelocity;
            }
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
     * Calculate the setting to use based on the inputs when in simple path mode
     * @return settings for left and right motor
     */
    private Setpoint calculateSimplePathModeSetpoint()
    {
        // get the desired left and right values from the driver.
        // note that position goals are in inches and velocity goals are in inches/second
        double leftVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainLeftVelocity);
        double rightVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainRightVelocity);

        leftVelocityGoal /= TuningConstants.DRIVETRAIN_PATH_LEFT_MAX_VELOCITY_INCHES_PER_SECOND;
        rightVelocityGoal /= TuningConstants.DRIVETRAIN_PATH_RIGHT_MAX_VELOCITY_INCHES_PER_SECOND;

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftVelocityPathGoal", leftVelocityGoal);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightVelocityPathGoal", rightVelocityGoal);

        // add in velocity as a type of feed-forward
        double leftGoal = leftVelocityGoal * TuningConstants.DRIVETRAIN_PATH_PID_LEFT_KV;
        double rightGoal = rightVelocityGoal * TuningConstants.DRIVETRAIN_PATH_PID_RIGHT_KV;

        // velocity being too high could put us over our max or under our min power levels
        leftGoal = this.applyPowerLevelRange(leftGoal);
        rightGoal = this.applyPowerLevelRange(rightGoal);

        this.assertPowerLevelRange(leftGoal, "left velocity (goal)");
        this.assertPowerLevelRange(rightGoal, "right velocity (goal)");

        if (this.usePID)
        {
            leftGoal *= TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS;
            rightGoal *= TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KS;
        }

        return new Setpoint(leftGoal, rightGoal);
    }

    /**
     * Calculate the setting to use based on the inputs when in path mode
     * @return settings for left and right motor
     */
    private Setpoint calculatePathModeSetpoint()
    {
        // get the desired left and right values from the driver.
        // note that position goals are in inches and velocity goals are in inches/second
        double leftPositionGoal = this.driver.getAnalog(AnalogOperation.DriveTrainLeftPosition);
        double rightPositionGoal = this.driver.getAnalog(AnalogOperation.DriveTrainRightPosition);
        double leftVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainLeftVelocity);
        double rightVelocityGoal = this.driver.getAnalog(AnalogOperation.DriveTrainRightVelocity);
        double headingCorrection = this.driver.getAnalog(AnalogOperation.DriveTrainHeadingCorrection);

        leftVelocityGoal /= TuningConstants.DRIVETRAIN_PATH_LEFT_MAX_VELOCITY_INCHES_PER_SECOND;
        rightVelocityGoal /= TuningConstants.DRIVETRAIN_PATH_RIGHT_MAX_VELOCITY_INCHES_PER_SECOND;

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftPositionPathGoal", leftPositionGoal);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightPositionPathGoal", rightPositionGoal);
        this.logger.logNumber(DriveTrainMechanism.LogName, "leftVelocityPathGoal", leftVelocityGoal);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightVelocityPathGoal", rightVelocityGoal);

        // use positional PID to get the relevant value
        double leftGoal = this.leftPID.calculatePosition(leftPositionGoal, this.leftPosition);
        double rightGoal = this.rightPID.calculatePosition(rightPositionGoal, this.rightPosition);

        // add in velocity as a type of feed-forward
        leftGoal += leftVelocityGoal * TuningConstants.DRIVETRAIN_PATH_PID_LEFT_KV;
        rightGoal += rightVelocityGoal * TuningConstants.DRIVETRAIN_PATH_PID_RIGHT_KV;

        // apply cross-coupling changes
        double leftPositionError = this.leftPID.getError();
        double rightPositionError = this.rightPID.getError();

        double positionErrorMagnitudeDelta = leftPositionError - rightPositionError;
        if (TuningConstants.DRIVETRAIN_USE_CROSS_COUPLING
            && !Helpers.WithinDelta(positionErrorMagnitudeDelta, 0.0, TuningConstants.DRIVETRAIN_CROSS_COUPLING_ZERO_ERROR_RANGE))
        {
            // add the delta times the coupling factor to the left, and subtract from the right
            // (if left error is greater than right error, left should be given some more power than right)
            leftGoal += TuningConstants.DRIVETRAIN_PATH_PID_LEFT_KCC * positionErrorMagnitudeDelta;
            rightGoal -= TuningConstants.DRIVETRAIN_PATH_PID_RIGHT_KCC * positionErrorMagnitudeDelta;
        }

        // apply heading correction
        if (TuningConstants.DRIVETRAIN_USE_HEADING_CORRECTION
            && headingCorrection != 0.0)
        {
            leftGoal += TuningConstants.DRIVETRAIN_PATH_LEFT_HEADING_CORRECTION * headingCorrection;
            rightGoal -= TuningConstants.DRIVETRAIN_PATH_RIGHT_HEADING_CORRECTION * headingCorrection;
        }

        // velocity plus position correction could put us over our max or under our min power levels
        leftGoal = this.applyPowerLevelRange(leftGoal);
        rightGoal = this.applyPowerLevelRange(rightGoal);

        this.assertPowerLevelRange(leftGoal, "left velocity (goal)");
        this.assertPowerLevelRange(rightGoal, "right velocity (goal)");

        if (this.usePID)
        {
            leftGoal *= TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS;
            rightGoal *= TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KS;
        }

        return new Setpoint(leftGoal, rightGoal);
    }

    /**
     * Calculate the setting to use based on the inputs when in position mode
     * @return settings for left and right motor
     */
    private Setpoint calculatePositionModeSetpoint()
    {
        // get the desired left and right values from the driver.
        double leftPositionGoal = this.driver.getAnalog(AnalogOperation.DriveTrainLeftPosition);
        double rightPositionGoal = this.driver.getAnalog(AnalogOperation.DriveTrainRightPosition);

        this.logger.logNumber(DriveTrainMechanism.LogName, "leftPositionGoal", leftPositionGoal);
        this.logger.logNumber(DriveTrainMechanism.LogName, "rightPositionGoal", rightPositionGoal);

        double leftPower;
        double rightPower;
        if (this.usePID)
        {
            // use positional PID to get the relevant value
            leftPower = this.leftPID.calculatePosition(leftPositionGoal, this.leftPosition);
            rightPower = this.rightPID.calculatePosition(rightPositionGoal, this.rightPosition);

            // apply cross-coupling changes
            double leftPositionError = this.leftPID.getError();
            double rightPositionError = this.rightPID.getError();

            double positionErrorMagnitudeDelta = leftPositionError - rightPositionError;
            if (TuningConstants.DRIVETRAIN_USE_CROSS_COUPLING
                && !Helpers.WithinDelta(positionErrorMagnitudeDelta, 0.0, TuningConstants.DRIVETRAIN_CROSS_COUPLING_ZERO_ERROR_RANGE))
            {
                // add the delta times the coupling factor to the left, and subtract from the right
                // (if left error is greater than right error, left should be given some more power than right)
                leftPower += TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KCC * positionErrorMagnitudeDelta;
                rightPower -= TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KCC * positionErrorMagnitudeDelta;

                // cross-coupling could put us over our max or under our min power levels
                leftPower = this.applyPowerLevelRange(leftPower);
                rightPower = this.applyPowerLevelRange(rightPower);
            }
        }
        else
        {
            // calculate a desired power level
            leftPower = leftPositionGoal - this.leftPosition;
            rightPower = rightPositionGoal - this.rightPosition;
            if (Math.abs(leftPower) < 0.1)
            {
                leftPower = 0.0;
            }

            if (Math.abs(rightPower) < 0.1)
            {
                rightPower = 0.0;
            }

            leftPower *= TuningConstants.DRIVETRAIN_LEFT_POSITIONAL_NON_PID_MULTIPLICAND;
            rightPower *= TuningConstants.DRIVETRAIN_RIGHT_POSITIONAL_NON_PID_MULTIPLICAND;

            // ensure that we are within our power level range, and then scale it down
            leftPower = this.applyPowerLevelRange(leftPower) * TuningConstants.DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
            rightPower = this.applyPowerLevelRange(rightPower) * TuningConstants.DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
        }

        this.assertPowerLevelRange(leftPower, "left velocity (goal)");
        this.assertPowerLevelRange(rightPower, "right velocity (goal)");

        if (this.usePID)
        {
            leftPower *= TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS;
            rightPower *= TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KS;
        }

        return new Setpoint(leftPower, rightPower);
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
