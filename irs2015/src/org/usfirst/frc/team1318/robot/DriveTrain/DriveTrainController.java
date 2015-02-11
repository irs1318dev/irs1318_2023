package org.usfirst.frc.team1318.robot.DriveTrain;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;

/**
 * Drivetrain controller.
 * The controller defines the logic that controls a mechanism given inputs (component) and operator-requested actions, and 
 * translates those into the abstract functions that should be applied to the outputs (component).
 * 
 * @author Will
 * 
 */
public class DriveTrainController implements IController
{
    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private IDriver driver;
    private IDriveTrainComponent component;

    private boolean usePID;
    private boolean usePositionalMode;
    private PIDHandler leftPID;
    private PIDHandler rightPID;

    private int prevLeftTicks;
    private int prevRightTicks;

    /**
     * Initializes a new DriveTrainController
     * @param operator to use to control the drive train
     * @param component to control
     * @param usePID indicates whether we should use PID control
     */
    public DriveTrainController(IDriver operator, IDriveTrainComponent component, boolean usePID)
    {
        this.driver = operator;
        this.component = component;
        this.usePID = usePID;
        this.usePositionalMode = false;

        this.createPIDHandler();

        this.prevLeftTicks = 0;
        this.prevRightTicks = 0;
    }

    /**
     * calculate the various outputs to use based on the inputs and apply them to the outputs for the relevant component
     */
    public void update()
    {
        // check our desired PID mode
        boolean newUsePositionalMode = this.driver.getDriveTrainPositionMode();
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

        // ensure that our algorithms are correct and don't give values outside
        // the appropriate range
        this.assertPowerLevelRange(leftPower, "left");
        this.assertPowerLevelRange(rightPower, "right");

        // apply the power settings to the drivetrain component
        this.component.setDriveTrainPower(leftPower, rightPower);
    }

    /**
     * stop the relevant component
     */
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
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KP_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KI_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KD_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_LEFT_KF_DEFAULT,
                    DriveTrainController.POWERLEVEL_MIN,
                    DriveTrainController.POWERLEVEL_MAX);

                this.rightPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KP_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KI_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KD_DEFAULT,
                    TuningConstants.DRIVETRAIN_POSITION_PID_RIGHT_KF_DEFAULT,
                    DriveTrainController.POWERLEVEL_MIN,
                    DriveTrainController.POWERLEVEL_MAX);
            }
            else
            {
                this.leftPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KP_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KI_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KD_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_LEFT_KF_DEFAULT,
                    DriveTrainController.POWERLEVEL_MIN,
                    DriveTrainController.POWERLEVEL_MAX);

                this.rightPID = new PIDHandler(
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KP_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KI_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KD_DEFAULT,
                    TuningConstants.DRIVETRAIN_VELOCITY_PID_RIGHT_KF_DEFAULT,
                    DriveTrainController.POWERLEVEL_MIN,
                    DriveTrainController.POWERLEVEL_MAX);
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
        boolean simpleDriveModeEnabled = this.driver.getDriveTrainSimpleMode();

        // get the X and Y values from the operator.  We expect these to be between -1.0 and 1.0,
        // with this value representing the forward velocity percentage and right turn percentage (of max speed)
        double xVelocity = this.driver.getDriveTrainXVelocity();
        double yVelocity = this.driver.getDriveTrainYVelocity();

        // calculate the distance from the joystick origin we are
        double radius = Math.sqrt(xVelocity * xVelocity + yVelocity * yVelocity);

        // adjust the intensity of the input
        xVelocity = this.adjustIntensity(xVelocity);
        yVelocity = this.adjustIntensity(yVelocity);

        // if we are outside of our dead zone, calculate desired power values
        if (radius > TuningConstants.DRIVETRAIN_DEAD_ZONE)
        {
            if (simpleDriveModeEnabled)
            {
                // simple drive enables either forward/back or in-place left/right turn only
                //
                //                   forward
                //               ---------------
                //               |      |      |
                //               |      |      |
                // In-place left |-------------| In-place right
                //               |      |      |
                //               |      |      |
                //               ---------------
                //                  backward
                //

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
                // advanced drive enables varying-degree turns.
                // math is derived using linear interpolation
                //
                //     a,1       1,1       1,a
                //      ---------------------
                //      |         |         |
                //      |   Q2    |   Q1    |
                //      |         |         |
                // -b,b |-------------------| b,-b
                //      |         |         |
                //      |   Q3    |   Q4    |
                //      |         |         |
                //      ---------------------
                //    -a,-1     -1,-1     -1,-a
                //
                // for x: 0 -> 1, power(x) = power(0) + x*(power(1) - power(0)) 
                // for y: 0 -> 1, power(x,y) = power(x,0) + y*(power(x,1) - power(x,0))

                if (xVelocity >= 0)
                {
                    if (yVelocity >= 0)
                    {
                        // Q1:
                        // y=1 => lp = 1.  rp = 1 + x*(a - 1)
                        // y=0 => lp = 0 + x*b = x*b.  rp = 0 + x*-b = -x*b
                        // lp = x*b + y*(1 - x*b)
                        // rp = x*-b + y*(1+x*(a-1) - x*-b)
                        leftVelocityGoal = xVelocity
                            * TuningConstants.DRIVETRAIN_B + yVelocity * (1 - xVelocity * TuningConstants.DRIVETRAIN_B);
                        rightVelocityGoal = -xVelocity
                            * TuningConstants.DRIVETRAIN_B + yVelocity
                            * (1 + xVelocity * (TuningConstants.DRIVETRAIN_A - 1) + xVelocity * TuningConstants.DRIVETRAIN_B);
                    }
                    else
                    {
                        // Q4:
                        // y=-1 => lp = -1.  rp = -1 + x*(-a - -1)  
                        // y=0  => lp = x*B.  rp = -x*B (see Q1)
                        // lp = x*B + -1*y*(-1 - x*B)
                        // rp = x*-B + -1*y*(-1+x*(-a - -1) - x*-B)
                        leftVelocityGoal = xVelocity
                            * TuningConstants.DRIVETRAIN_B - yVelocity * (-1 - xVelocity * TuningConstants.DRIVETRAIN_B);
                        rightVelocityGoal = -xVelocity
                            * TuningConstants.DRIVETRAIN_B - yVelocity
                            * (-1 + xVelocity * (-TuningConstants.DRIVETRAIN_A + 1) + xVelocity * TuningConstants.DRIVETRAIN_B);
                    }
                }
                else
                {
                    if (yVelocity >= 0)
                    {
                        // Q2:
                        // y=1 => lp = 1 + -1*x*(a - 1) = 1 - x*(a - 1).  rp = 1
                        // y=0 => lp = 0 + -1*x*(-b - 0) = x*b.  rp = 0 + -1*x*(b - 0) = -x*b
                        // lp = x*b + y*(1 - x*(a-1) - x*b)
                        // rp = -x*b + y*(1 - -x*B)
                        leftVelocityGoal = xVelocity
                            * TuningConstants.DRIVETRAIN_B + yVelocity
                            * (1 - xVelocity * (TuningConstants.DRIVETRAIN_A - 1) - xVelocity * TuningConstants.DRIVETRAIN_B);
                        rightVelocityGoal = -xVelocity
                            * TuningConstants.DRIVETRAIN_B + yVelocity * (1 + xVelocity * TuningConstants.DRIVETRAIN_B);
                    }
                    else
                    {
                        // Q3:
                        // y=-1 => lp = -1 + -1*x*(-a - -1) = -1 - x*(-a + 1).  rp = -1 
                        // y=0  => lp = x*b.  rp = -x*b (see Q2) 
                        // lp = x*b + -1*y*(-1 - x*(-a + 1) - x*b)
                        // rp = -x*b + -1*y*(-1 - -x*b)
                        leftVelocityGoal = xVelocity
                            * TuningConstants.DRIVETRAIN_B - yVelocity
                            * (-1 - xVelocity * (-TuningConstants.DRIVETRAIN_A + 1) - xVelocity * TuningConstants.DRIVETRAIN_B);
                        rightVelocityGoal = -xVelocity
                            * TuningConstants.DRIVETRAIN_B - yVelocity * (-1 + xVelocity * TuningConstants.DRIVETRAIN_B);
                    }
                }
            }
        }

        // ensure that our algorithms are correct and don't give values outside
        // the appropriate range
        this.assertPowerLevelRange(leftVelocityGoal, "left velocity (goal)");
        this.assertPowerLevelRange(rightVelocityGoal, "right velocity (goal)");

        // decrease the desired velocity based on the configured max power level
        leftVelocityGoal = leftVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;
        rightVelocityGoal = rightVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;

        // convert velocity goal to power level...
        double leftPower;
        double rightPower;
        if (this.usePID)
        {
            leftPower =
                this.leftPID.calculate(
                    leftVelocityGoal,
                    (currentLeftTicks - this.prevLeftTicks));

            rightPower =
                this.rightPID.calculate(
                    rightVelocityGoal,
                    (currentRightTicks - this.prevRightTicks));
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
        this.assertPowerLevelRange(leftPower, "left velocity (goal)");
        this.assertPowerLevelRange(rightPower, "right velocity (goal)");

        this.prevLeftTicks = currentLeftTicks;
        this.prevRightTicks = currentRightTicks;

        return new PowerSetting(leftPower, rightPower);
    }

    /**
     * Calculate the power setting to use based on the inputs when in position mode
     * @return power settings for left and right motor
     */
    private PowerSetting calculatePositionModePowerSetting()
    {
        // get the desired left and right values from the driver.
        double leftPosition = this.driver.getDriveTrainLeftPosition();
        double rightPosition = this.driver.getDriveTrainRightPosition();

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
            leftPower = this.leftPID.calculate(leftPosition, leftDistance);
            rightPower = this.rightPID.calculate(rightPosition, rightDistance);
        }
        else
        {
            // calculate a desired 
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
     * Adjust the intensity of the input value
     * @param value to adjust
     * @return adjusted value
     */
    private double adjustIntensity(double value)
    {
        // we will use simple quadratic scaling to adjust input intensity
        if (value < 0)
        {
            return -value * value;
        }
        else
        {
            return value * value;
        }
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
