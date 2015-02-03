package org.usfirst.frc.team1318.robot.Intake;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;

/* buttons 
 * up
 * down
 * toggle up and down
 * press and hold for in
 * press and hold for out
 * */

public class IntakeController implements IController
{
    private final IntakeComponent intake;
    private final IDriver driver;
    private static final double INTAKE_SPEED = .7;
    private static final int MOTOR_FORWARD = 1;
    private static final int MOTOR_STOP = 0;
    private static final int MOTOR_REVERSE = -1;

    // state variables
    // true = outward(motor)/extending(piston), false = inward/retracting
    private boolean solenoidLeftState;
    private boolean solenoidRightState;
    private int motorState;

    public IntakeController(IDriver driver, IntakeComponent intake)
    {
        this.intake = intake;
        this.driver = driver;
    }

    public void update()
    {
        // gets joystick input and translates it into wanted motor state
        if (this.driver.getIntakeForwardButton())
        {
            motorState = MOTOR_FORWARD;
        }
        else if (this.driver.getIntakeBackwardButton())
        {
            motorState = MOTOR_REVERSE;
        }
        else
        {
            motorState = MOTOR_STOP;
        }

        // gets joystick input and translates it into wanted solenoid state
        if (this.driver.getIntakeUpButton())
        {
            this.solenoidLeftState = true;
            this.solenoidRightState = true;
        }
        if (this.driver.getIntakeDownButton())
        {
            this.solenoidLeftState = false;
            this.solenoidRightState = false;
        }

        if (this.driver.getIntakeLeftExtendOverride())
        {
            this.solenoidLeftState = true;
        }
        if (this.driver.getIntakeLeftRetractOverride())
        {
            this.solenoidLeftState = false;
        }

        if (this.driver.getIntakeRightExtendOverride())
        {
            this.solenoidRightState = true;
        }
        if (this.driver.getIntakeRightRetractOverride())
        {
            this.solenoidRightState = false;
        }

        // sets IntakeComponent using solenoid and motor states
        intake.setIntake(solenoidLeftState, solenoidRightState);
        intake.setIntakeMotorSpeed(motorState * INTAKE_SPEED);
    }

    public void stop()
    {
        this.intake.setIntakeMotorSpeed(0);
    }
}

/**
 * Toggles the left intake solenoid
 * If the piston is extending, it will be switched to retracting
 * If the piston is off or retracting, it will be switched to extending
*/

/**
 * Toggles the right intake solenoid
 * If the piston is extending, it will be switched to retracting
 * If the piston is off or retracting, it will be switched to extending
*/
