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
    private IntakeComponent intake;
    private IDriver operator;
    private static final double INTAKE_SPEED = .7;

    public IntakeController(IDriver operator, IntakeComponent intake)
    {
        this.intake = intake;
        this.operator = operator;
    }

    // private boolean isToggleUp;

    public void update()
    {
        //Raises the intake arm
        if (operator.getIntakeUpButton())
        {
            intake.setIntake(true);
        }

        //Lowers the intake arm
        if (operator.getIntakeDownButton())
        {
            intake.setIntake(false);
        }

        //Toggles the right intake arm solenoid
        if (operator.getIntakeRightToggleOverride())
        {
            intake.toggleRightIntake();
        }

        //Toggles the left intake arm solenoid
        if (operator.getIntakeLeftToggleOverride())
        {
            intake.toggleLeftIntake();
        }

        //Rotates the intake wheels
        if (operator.getIntakeForwardButton())
        {
            intake.setIntakeMotorSpeed(INTAKE_SPEED);
        }

        //Rotates wheels outwards while held
        else if (operator.getIntakeBackwardButton())
        {
            intake.setIntakeMotorSpeed(-INTAKE_SPEED);
        }

        //Turns off the motors
        else
        {
            intake.setIntakeMotorSpeed(0.0);

        }
    }

    public void stop()
    {
        intake.setIntakeMotorSpeed(0);
    }

}
