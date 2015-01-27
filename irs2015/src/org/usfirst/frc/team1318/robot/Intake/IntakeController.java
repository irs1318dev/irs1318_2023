package org.usfirst.frc.team1318.robot.Intake;

import org.usfirst.frc.team1318.robot.Common.IDriver;

/* buttons 
 * up
 * down
 * toggle up and down
 * press and hold for in
 * press and hold for out
 * */

public class IntakeController implements IDriver
{
    private IntakeComponent intake;

    // private boolean isToggleUp;

    public void update()
    {
        //Raises the intake arm
        if (getIntakeUpButton())
        {
            intake.raiseIntake();
        }

        //Lowers the intake arm
        if (getIntakeDownButton())
        {
            intake.lowerIntake();
        }

        //Toggles the right intake arm solenoid
        if (getIntakeRightToggleOverride())
        {
            intake.toggleRightIntake();
        }

        //Toggles the left intake arm solenoid
        if (getIntakeLeftToggleOverride())
        {
            intake.toggleLeftIntake();
        }

        //Rotates the intake wheels inwards while held
        if (getIntakeForwardButton())
        {
            intake.intakeIn();
        }

        //Rotates wheels outwards while held
        else if (getIntakeBackwardButton())
        {
            intake.intakeOut();
        }

        //Turns off the motors
        else
        {
            intake.intakeOff();

        }
    }

    @Override
    public void stop()
    {
        intake.intakeOff();
    }

    @Override
    public double getDriveTrainXVelocity()
    {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getDriveTrainYVelocity()
    {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getDriveTrainSimpleMode()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public double getDriveTrainLeftPosition()
    {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getDriveTrainRightPosition()
    {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getDriveTrainPositionMode()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getElevatorMacroButton()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getElevatorHeight0Button()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getElevatorHeight1Button()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getElevatorHeight2Button()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getElevatorHeight3Button()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getElevatorHeight4Button()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getElevatorHeight5Button()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getElevatorHeight6Button()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getElevatorHeight7Button()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public double getElevatorOverride()
    {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getArmMacroToggle()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getArmExtenderToggleOverride()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getArmTiltToggleOverride()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getArmTromboneToggleOverride()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeUpButton()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeDownButton()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeRightToggleOverride()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeLeftToggleOverride()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeForwardButton()
    {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getIntakeBackwardButton()
    {
        // TODO Auto-generated method stub
        return false;
    }
}
