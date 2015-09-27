package org.usfirst.frc.team1318.robot.Driver.States;

import org.usfirst.frc.team1318.robot.Driver.Buttons.ClickButton;
import org.usfirst.frc.team1318.robot.Driver.Buttons.IButton;
import org.usfirst.frc.team1318.robot.Driver.Buttons.SimpleButton;
import org.usfirst.frc.team1318.robot.Driver.Buttons.ToggleButton;
import org.usfirst.frc.team1318.robot.Driver.Descriptions.DigitalOperationDescription;

import edu.wpi.first.wpilibj.Joystick;

public class DigitalOperationState extends OperationState
{
    private final IButton button;

    public DigitalOperationState(DigitalOperationDescription description)
    {
        super(description);

        switch (description.getButtonType())
        {
            case Simple:
                this.button = new SimpleButton();
                break;

            case Click:
                this.button = new ClickButton();
                break;

            case Toggle:
                this.button = new ToggleButton();
                break;

            default:
                throw new RuntimeException("unexpected button type " + description.getButtonType().toString());
        }
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
    }

    public boolean getState()
    {
        return this.button.isActivated();
    }
}
