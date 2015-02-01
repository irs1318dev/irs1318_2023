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
    private final IDriver operator;
    private static final double INTAKE_SPEED = .7;
    private static final int MOTOR_FORWARD = 1;
    private static final int MOTOR_STOP = 0;
    private static final int MOTOR_REVERSE = -1;

    // state variables
    // true = outward(motor)/extending(piston), false = inward/retracting
    private boolean solenoidLeftState;
    private boolean solenoidRightState;
    private int motorState;

    public IntakeController(IDriver operator, IntakeComponent intake)
    {
        this.intake = intake;
        this.operator = operator;
    }

    public void update()
    {
        // gets joystick input and translates it into wanted motor state
        if (this.operator.getIntakeForwardButton())
        {
            motorState = MOTOR_FORWARD;
        }
        else if (this.operator.getIntakeBackwardButton())
        {
            motorState = MOTOR_REVERSE;
        }
        else
        {
            motorState = MOTOR_STOP;
        }

        // gets joystick input and translates it into wanted solenoid state
        if (this.operator.getIntakeUpButton())
        {
            this.solenoidLeftState = true;
            this.solenoidRightState = true;
        }
        else if (this.operator.getIntakeDownButton())
        {
            this.solenoidLeftState = false;
            this.solenoidRightState = false;
        }

        if (this.operator.getIntakeLeftToggleOverride())
        {
            this.solenoidLeftState = !this.solenoidLeftState;
        }

        if (this.operator.getIntakeRightToggleOverride())
        {
            this.solenoidRightState = !this.solenoidRightState;
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
