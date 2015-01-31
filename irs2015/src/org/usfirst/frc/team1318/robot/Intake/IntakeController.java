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

    public IntakeController(IDriver operator, IntakeComponent intake)
    {
        this.intake = intake;
        this.operator = operator;
    }

    // private boolean isToggleUp;

    public void update()
    {
        //Raises the intake arm
        if (this.operator.getIntakeUpButton())
        {
            this.intake.setIntake(true);
        }

        //Lowers the intake arm
        if (this.operator.getIntakeDownButton())
        {
            this.intake.setIntake(false);
        }

        //Toggles the right intake arm solenoid
        if (this.operator.getIntakeRightToggleOverride())
        {
            this.intake.toggleRightIntake();
        }

        //Toggles the left intake arm solenoid
        if (this.operator.getIntakeLeftToggleOverride())
        {
            this.intake.toggleLeftIntake();
        }

        //Rotates the intake motors when forward button is held
        if (this.operator.getIntakeForwardButton())
        {
            this.intake.setIntakeMotorSpeed(IntakeController.INTAKE_SPEED);
        }

        //Rotates wheels outwards when backward button is held
        else if (this.operator.getIntakeBackwardButton())
        {
            this.intake.setIntakeMotorSpeed(-IntakeController.INTAKE_SPEED);
        }

        //Turns off the motors if not button is held
        else
        {
            this.intake.setIntakeMotorSpeed(0.0);
        }
    }

    public void stop()
    {
        this.intake.setIntakeMotorSpeed(0);
        this.intake.stopSolenoids();
    }
}
