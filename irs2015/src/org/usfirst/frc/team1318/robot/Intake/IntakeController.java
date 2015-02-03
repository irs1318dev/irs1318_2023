package org.usfirst.frc.team1318.robot.Intake;

import org.usfirst.frc.team1318.robot.TuningConstants;
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
    private double motorSpeed;

    // state variables
    // true = outward(motor)/extending(piston), false = inward/retracting
    private boolean solenoidLeftState;
    private boolean solenoidRightState;

    public IntakeController(IDriver operator, IntakeComponent intake)
    {
        this.intake = intake;
        this.operator = operator;
        this.motorSpeed = 0;
        this.solenoidLeftState = false;
        this.solenoidRightState = false;
    }

    public void update()
    {
        // gets joystick input and translates it into wanted motor state
        if (this.operator.getIntakeForwardButton())
        {
            this.motorSpeed = TuningConstants.INTAKE_MOTOR_SPEED;
        }
        else if (this.operator.getIntakeBackwardButton())
        {
            this.motorSpeed = -TuningConstants.INTAKE_MOTOR_SPEED;
        }
        else
        {
            this.motorSpeed = 0;
        }

        // gets joystick input and translates it into wanted solenoid state
        if (this.operator.getIntakeUpButton())
        {
            this.solenoidLeftState = true;
            this.solenoidRightState = true;
        }

        if (this.operator.getIntakeDownButton())
        {
            this.solenoidLeftState = false;
            this.solenoidRightState = false;
        }

        if (this.operator.getIntakeLeftExtendOverride())
        {
            this.solenoidLeftState = true;
        }

        if (this.operator.getIntakeLeftRetractOverride())
        {
            this.solenoidLeftState = false;
        }

        if (this.operator.getIntakeRightExtendOverride())
        {
            this.solenoidRightState = true;
        }

        if (this.operator.getIntakeRightRetractOverride())
        {
            this.solenoidRightState = false;
        }

        // sets IntakeComponent using solenoid and motor states
        this.intake.setLeftIntake(this.solenoidLeftState);
        this.intake.setRightIntake(this.solenoidRightState);
        this.intake.setIntakeMotorSpeed(this.motorSpeed);
    }

    public void stop()
    {
        this.intake.setIntakeMotorSpeed(0);
    }
}
