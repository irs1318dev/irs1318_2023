package org.usfirst.frc.team1318.robot.DriveTrain;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;
import org.usfirst.frc.team1318.robot.Driver.Driver;
import org.usfirst.frc.team1318.robot.Driver.Operation;

/**
 * Drivetrain controller.
 * The controller defines the logic that controls a mechanism given inputs (component) and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs (component).
 * 
 */
public class DriveTrainController implements IController
{
    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private Driver driver;
    private DriveTrainComponent component;

    private boolean usePID;
    private boolean usePositionalMode;
    private PIDHandler leftPID;
    private PIDHandler rightPID;

    /**
     * Initializes a new DriveTrainController
     * @param component to control
     * @param usePID indicates whether we should use PID control
     */
    public DriveTrainController(DriveTrainComponent component, boolean usePID)
    {
        this.component = component;
        this.usePID = usePID;
        this.usePositionalMode = false;

        this.createPIDHandler();
    }

    /**
     * set the driver that the controller should use
     * @param driver to use
     */
    @Override
    public void setDriver(Driver driver)
    {
        this.driver = driver;
    }

    public void setVelocityPIDMode()
    {
        if (!this.usePID || this.usePositionalMode)
        {
            this.usePID = true;
            this.usePositionalMode = false;

            this.createPIDHandler();
        }
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant component
     */
    @Override
    public void update()
    {
        // check our desired PID mode
        boolean newUsePositionalMode = this.driver.getDigital(Operation.DriveTrainUsePositionalMode);
        if (newUsePositionalMode != this.usePositionalMode)
        {
            this.usePositionalMode = newUsePositionalMode;

            // re-create PID handler
            this.createPIDHandler();
        }

        // calculate desired power setting for the current mode
        PowerSetting powerSetting;
        if (!this.usePositionalMode)
        {
            powerSetting = this.calculateVelocityModePowerSetting();
        }
        else
        {
            powerSetting = this.calculatePositionModePowerSetting();
        }

        double leftPower = powerSetting.getLeftPower();
        double rightPower = powerSetting.getRightPower();

        if (leftPower > 0)
        {
            leftPower /= TuningConstants.DRIVETRAIN_REVERSE_LEFT_SCALE_FACTOR;
        }

        if (rightPower > 0)
        {
            rightPower /= TuningConstants.DRIVETRAIN_REVERSE_RIGHT_SCALE_FACTOR;
        }

        leftPower = Math.min(leftPower, 1);
        leftPower = Math.max(leftPower, -1);
        rightPower = Math.min(rightPower, 1);
        rightPower = Math.max(rightPower, -1);

        // apply the power settings to the drivetrain component
        this.component.setDriveTrainPower(leftPower, rightPower);
    }

    /**
     * stop the relevant component
     */
    @Override
    public void stop()
    {
        this.component.setDriveTrainPower(0.0, 0.0);
    }

    /**
     * create a PIDHandler based on our current settings
     */
    private void createPIDHandler()
    {
        if (!this.usePID)
        {
            this.leftPID = null;
            this.rightPID = null;
        }
        else
        {
            if (this.usePositionalMode)
            {
                this.leftPID = new PIDHandler(
                    "dt.leftPID",
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KP_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KI_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KD_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KF_DEFAULT,
                    -TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL);

                this.rightPID = new PIDHandler(
                    "dt.rightPID",
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KP_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KI_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KD_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KF_DEFAULT,
                    -TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_POSITIONAL_MAX_POWER_LEVEL);
            }
            else
            {
                this.leftPID = new PIDHandler(
                    "dt.leftPID",
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KP_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KI_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KD_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KS_DEFAULT,
                    -TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL);

                this.rightPID = new PIDHandler(
                    "dt.rightPID",
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KP_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KI_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KD_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KF_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KS_DEFAULT,
                    -TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL,
                    TuningConstants.DRIVETRAIN_VELOCITY_MAX_POWER_LEVEL);
                ;
            }
        }
    }

    /**
     * Calculate the power setting to use based on the inputs when in velocity mode
     * @return power settings for left and right motor
     */
    private PowerSetting calculateVelocityModePowerSetting()
    {
        // velocity goals represent the desired percentage of the max velocity
        double leftVelocityGoal = 0.0;
        double rightVelocityGoal = 0.0;

        // read the encoder distance just in case we want it output in smart dashboard
        this.component.getLeftEncoderDistance();
        this.component.getRightEncoderDistance();
        this.component.getLeftEncoderVelocity();
        this.component.getRightEncoderVelocity();

        int currentLeftTicks = this.component.getLeftEncoderTicks();
        int currentRightTicks = this.component.getRightEncoderTicks();

        // get a value indicating that we should be in simple mode...
        boolean simpleDriveModeEnabled = this.driver.getDigital(Operation.DriveTrainSimpleMode);

        // get the X and Y values from the operator.  We expect these to be between -1.0 and 1.0,
        // with this value representing the forward velocity percentage and right turn percentage (of max speed)
        double xVelocity = this.driver.getAnalog(Operation.DriveTrainTurn);
        double yVelocity = this.driver.getAnalog(Operation.DriveTrainMoveForward);

        // adjust for joystick deadzone
        xVelocity = this.adjustForDeadZone(xVelocity, TuningConstants.DRIVETRAIN_X_DEAD_ZONE);
        yVelocity = this.adjustForDeadZone(yVelocity, TuningConstants.DRIVETRAIN_Y_DEAD_ZONE);

        // adjust the intensity of the input
        if (simpleDriveModeEnabled)
        {
            if (Math.abs(yVelocity) < Math.abs(xVelocity))
            {
                // in-place turn
                leftVelocityGoal = xVelocity;
                rightVelocityGoal = -xVelocity;
            }
            else
            {
                // forward/backward
                leftVelocityGoal = yVelocity;
                rightVelocityGoal = yVelocity;
            }
        }
        else
        {
            leftVelocityGoal = (TuningConstants.DRIVETRAIN_K1 * yVelocity) + (TuningConstants.DRIVETRAIN_K2 * xVelocity);
            rightVelocityGoal = (TuningConstants.DRIVETRAIN_K1 * yVelocity) + (-TuningConstants.DRIVETRAIN_K2 * xVelocity);
        }

        // decrease the desired velocity based on the configured max power level
        leftVelocityGoal = leftVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;
        rightVelocityGoal = rightVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;

        // convert velocity goal to power level...
        double leftPower;
        double rightPower;
        if (this.usePID)
        {
            leftPower = this.leftPID.calculateVelocity(
                leftVelocityGoal,
                currentLeftTicks);

            rightPower = this.rightPID.calculateVelocity(
                rightVelocityGoal,
                currentRightTicks);
        }
        else
        {
            leftPower = leftVelocityGoal;
            rightPower = rightVelocityGoal;
        }

        // ensure that our algorithms are correct and don't give values outside
        // the appropriate range
        leftPower = this.applyPowerLevelRange(leftPower);
        rightPower = this.applyPowerLevelRange(rightPower);

        return new PowerSetting(leftPower, rightPower);
    }

    /**
     * Calculate the power setting to use based on the inputs when in position mode
     * @return power settings for left and right motor
     */
    private PowerSetting calculatePositionModePowerSetting()
    {
        // get the desired left and right values from the driver.
        double leftPosition = this.driver.getAnalog(Operation.DriveTrainLeftPosition);
        double rightPosition = this.driver.getAnalog(Operation.DriveTrainRightPosition);

        // get the current encoder distance from the component.
        double leftDistance = this.component.getLeftEncoderDistance();
        double rightDistance = this.component.getRightEncoderDistance();

        // read the encoder velocity just in case we want it output in smart dashboard
        this.component.getLeftEncoderVelocity();
        this.component.getRightEncoderVelocity();

        double leftPower;
        double rightPower;
        if (this.usePID)
        {
            // use positional PID to get the relevant value
            leftPower = this.leftPID.calculatePosition(leftPosition, leftDistance);
            rightPower = this.rightPID.calculatePosition(rightPosition, rightDistance);
        }
        else
        {
            // calculate a desired power level
            leftPower = leftPosition - leftDistance;
            rightPower = rightPosition - rightDistance;
            if (Math.abs(leftPower) < 0.1)
            {
                leftPower = 0.0;
            }

            if (Math.abs(rightPower) < 0.1)
            {
                rightPower = 0.0;
            }

            // ensure that we are within our power level range, and then scale it down
            leftPower = this.applyPowerLevelRange(leftPower) * TuningConstants.DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
            rightPower = this.applyPowerLevelRange(rightPower) * TuningConstants.DRIVETRAIN_MAX_POWER_POSITIONAL_NON_PID;
        }

        this.assertPowerLevelRange(leftPower, "left velocity (goal)");
        this.assertPowerLevelRange(rightPower, "right velocity (goal)");

        return new PowerSetting(leftPower, rightPower);
    }

    /**
     * Adjust the velocity as a part of dead zone calculation
     * @param velocity to adjust
     * @param deadZone to consider
     * @return adjusted velocity for deadZone
     */
    private double adjustForDeadZone(double velocity, double deadZone)
    {
        if (velocity < deadZone && velocity > -deadZone)
        {
            return 0.0;
        }

        double sign = 1.0;
        if (velocity < 0.0)
        {
            sign = -1.0;
        }

        // scale so that we have the area just outside the deadzone be the starting point
        return (velocity - sign * deadZone) / (1 - deadZone);
    }

    /**
     * Assert that the power level is within the required range
     * @param powerLevel to verify
     * @param side indicator for the exception message if incorrect
     */
    private void assertPowerLevelRange(double powerLevel, String side)
    {
        if (powerLevel < DriveTrainController.POWERLEVEL_MIN)
        {
            throw new RuntimeException(side + " power level too low!");
        }

        if (powerLevel > DriveTrainController.POWERLEVEL_MAX)
        {
            throw new RuntimeException(side + " power level too high!");
        }
    }

    /**
     * Reset the power level to be within the required range
     * @param powerLevel to reset
     * @return power level
     */
    private double applyPowerLevelRange(double powerLevel)
    {
        if (powerLevel < DriveTrainController.POWERLEVEL_MIN)
        {
            return DriveTrainController.POWERLEVEL_MIN;
        }

        if (powerLevel > DriveTrainController.POWERLEVEL_MAX)
        {
            return DriveTrainController.POWERLEVEL_MAX;
        }

        return powerLevel;
    }

    /**
     * Simple holder of power setting information for the left and right motor
     * (This exists only to allow splitting out common code and have only one return value, because Java doesn't support multi-return)
     */
    private class PowerSetting
    {
        private double leftPower;
        private double rightPower;

        /**
         * Initializes a new PowerSetting
         * @param leftPower to apply
         * @param rightPower to apply
         */
        public PowerSetting(double leftPower, double rightPower)
        {
            this.leftPower = leftPower;
            this.rightPower = rightPower;
        }

        /**
         * gets the left power setting 
         * @return value between -1.0 and 1.0
         */
        public double getLeftPower()
        {
            return this.leftPower;
        }

        /**
         * gets the right power setting 
         * @return value between -1.0 and 1.0
         */
        public double getRightPower()
        {
            return this.rightPower;
        }
    }
}
