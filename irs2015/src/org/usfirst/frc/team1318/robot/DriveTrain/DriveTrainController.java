package org.usfirst.frc.team1318.robot.DriveTrain;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.Timer;

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
    private DriveTrainMacroData macroData;
    private final Timer timer;
    private Double startTime;

    private static final double POWERLEVEL_MIN = -1.0;
    private static final double POWERLEVEL_MAX = 1.0;

    private IDriver driver;
    private IDriveTrainComponent component;

    private boolean usePID;
    private boolean usePositionalMode;
    private PIDHandler leftPID;
    private PIDHandler rightPID;

    /**
     * Initializes a new DriveTrainController
     * @param operator to use to control the drive train
     * @param component to control
     * @param usePID indicates whether we should use PID control
     */
    public DriveTrainController(IDriver operator, IDriveTrainComponent component, DriveTrainMacroData driveTrainMacroData, boolean usePID)
    {
        this.driver = operator;
        this.component = component;
        this.usePID = usePID;
        this.usePositionalMode = false;
        this.macroData = driveTrainMacroData;
        this.timer = new Timer();

        this.timer.start();
        this.startTime = this.timer.get();

        this.createPIDHandler();
    }

    @Override
    public void setDriver(IDriver driver)
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
        //        this.component.getProximitySensorFront();
        //        this.component.getProximitySensorBack();

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

    public void setMacroData(DriveTrainMacroData macroData)
    {
        this.macroData = macroData;
    }

    private PowerSetting runCollectCansFromStepMacro()
    {
        PowerSetting result = new PowerSetting(0, 0);
        switch (this.macroData.state)
        {
            case STATE_0_WAIT_FOR_PRESS:
                if (this.driver.getDriveTrainCollectCansFromStepMacro())
                {
                    this.macroData.state = DriveTrainMacroData.MacroStates.STATE_1_DRIVE_BACK;
                    this.startTime = this.timer.get();
                    this.macroData.setRunningMacro(true);

                    this.macroData.setExtenderState(true);
                    this.macroData.setTiltState(false);
                    this.macroData.setTromboneState(true);
                }
                this.macroData.setRunningMacro(false);
                break;
            case STATE_1_DRIVE_BACK:
                if (this.timer.get() < this.startTime + this.macroData.DRIVE_BACK_TIME_1)
                {
                    result = new PowerSetting(0.0, 0.17);
                }
                else
                {
                    this.startTime = this.timer.get();
                    this.macroData.setTiltState(true);
                    this.macroData.state = DriveTrainMacroData.MacroStates.STATE_2_SETTLE_WAIT;
                }
                break;
            case STATE_2_SETTLE_WAIT:

        }
        return result;
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
        boolean simpleDriveModeEnabled = false;//this.driver.getDriveTrainSimpleMode();

        // get the X and Y values from the operator.  We expect these to be between -1.0 and 1.0,
        // with this value representing the forward velocity percentage and right turn percentage (of max speed)
        double xVelocity = this.driver.getDriveTrainXVelocity();
        double yVelocity = this.driver.getDriveTrainYVelocity();

        // adjust for joystick deadzone
        xVelocity = this.adjustForDeadZone(xVelocity, TuningConstants.DRIVETRAIN_X_DEAD_ZONE);
        yVelocity = this.adjustForDeadZone(yVelocity, TuningConstants.DRIVETRAIN_Y_DEAD_ZONE);

        if (xVelocity == 0 || yVelocity == 0)
        {
            PowerSetting temp = this.runCollectCansFromStepMacro();
            xVelocity = temp.leftPower;
            yVelocity = temp.rightPower;
        }
        else
        {
            this.macroData.state = DriveTrainMacroData.MacroStates.STATE_0_WAIT_FOR_PRESS;
        }

        // adjust the intensity of the input
        //        xVelocity = this.adjustIntensity(xVelocity);
        //        yVelocity = this.adjustIntensity(yVelocity);

        //        if (simpleDriveModeEnabled)
        //        {
        //            // simple drive enables either forward/back or in-place left/right turn only
        //            //
        //            //                   forward
        //            //               ---------------
        //            //               |      |      |
        //            //               |      |      |
        //            // In-place left |-------------| In-place right
        //            //               |      |      |
        //            //               |      |      |
        //            //               ---------------
        //            //                  backward
        //            //
        //
        //            if (Math.abs(yVelocity) < Math.abs(xVelocity))
        //            {
        //                // in-place turn
        //                leftVelocityGoal = xVelocity;
        //                rightVelocityGoal = -xVelocity;
        //            }
        //            else
        //            {
        //                // forward/backward
        //                leftVelocityGoal = yVelocity;
        //                rightVelocityGoal = yVelocity;
        //            }
        //        }
        //        else
        //        {
        double K1 = 1.5;
        double K2 = .4;
        double K3 = K1;
        double K4 = -K2;

        leftVelocityGoal = (K1 * yVelocity) + (K2 * xVelocity);
        rightVelocityGoal = (K3 * yVelocity) + (K4 * xVelocity);
        //        }

        // decrease the desired velocity based on the configured max power level
        leftVelocityGoal = leftVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;
        rightVelocityGoal = rightVelocityGoal * TuningConstants.DRIVETRAIN_MAX_POWER_LEVEL;

        // convert velocity goal to power level...
        double leftPower;
        double rightPower;
        if (this.usePID)
        {
            leftPower =
                this.leftPID.calculateVelocity(
                    leftVelocityGoal,
                    currentLeftTicks);

            SmartDashboardLogger.putNumber("leftVelocityGoal", leftVelocityGoal);
            SmartDashboardLogger.putNumber("leftPower", leftPower);

            rightPower =
                this.rightPID.calculateVelocity(
                    rightVelocityGoal,
                    currentRightTicks);

            SmartDashboardLogger.putNumber("rightVelocityGoal", rightVelocityGoal);
            SmartDashboardLogger.putNumber("rightPower", rightPower);
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
            leftPower = this.leftPID.calculatePosition(leftPosition, leftDistance);
            rightPower = this.rightPID.calculatePosition(rightPosition, rightDistance);
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
            //throw new RuntimeException(side + " power level too low!");
        }

        if (powerLevel > DriveTrainController.POWERLEVEL_MAX)
        {
            //throw new RuntimeException(side + " power level too high!");
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
        // Jim prefers linear
        return value;

        // we will use simple quadratic scaling to adjust input intensity
        //        if (value < 0)
        //        {
        //            return -value * value;
        //        }
        //        else
        //        {
        //            return value * value;
        //        }
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
