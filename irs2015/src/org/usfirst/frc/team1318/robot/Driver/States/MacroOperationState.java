package org.usfirst.frc.team1318.robot.Driver.States;

import org.usfirst.frc.team1318.robot.Driver.Buttons.ClickButton;
import org.usfirst.frc.team1318.robot.Driver.Buttons.IButton;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.DigitalOperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.MacroOperationDescription;
import org.usfirst.frc.team1318.robot.Driver.Macros.MacroTask;

import edu.wpi.first.wpilibj.Joystick;

public class MacroOperationState extends OperationState
{
    private final IButton button;
    private MacroTask task;

    public MacroOperationState(MacroOperationDescription description)
    {
        super(description);

        this.button = new ClickButton();
        this.task = null;
    }

    /**
     * Sets whether the current operation is being interrupted by a macro
     * @param enable value of true indicates that we are interrupted
     */
    @Override
    public void setInterrupt(boolean enable)
    {
    }

    /**
     * Update the operation state based on the driver and co-driver joysticks 
     * @param driver joystick to update from
     * @param coDriver joystick to update from
     */
    @Override
    public void update(Joystick driver, Joystick coDriver)
    {
        DigitalOperationDescription description = (DigitalOperationDescription)this.getDescription();

        Joystick relevantJoystick;
        int relevantButton;
        switch (description.getUserInputDevice())
        {
            case None:
                return;

            case Driver:
                relevantJoystick = driver;
                break;

            case CoDriver:
                relevantJoystick = coDriver;
                break;

            default:
                throw new RuntimeException("unexpected user input device " + description.getUserInputDevice().toString());
        }

        relevantButton = description.getUserInputDeviceButton();

        this.button.updateState(relevantJoystick.getRawButton(relevantButton));

        if (this.button.isActivated())
        {
            if (this.task == null)
            {
                // start task
                this.task = ((MacroOperationDescription)this.getDescription()).constructMacroTask();
                this.task.begin();
            }
            else
            {
                // cancel task:
                this.task.stop();
            }
        }
        else if (this.task != null)
        {
            if (this.task.hasCompleted())
            {
                this.task.end();
                this.task = null;
            }
            else
            {
                this.task.update();
            }
        }
    }

    public boolean isRunning()
    {
        return this.task != null;
    }
}
