package org.usfirst.frc.team1318.robot.Intake;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Talon;

/* buttons 
 * up
 * down
 * toggle left 
 * toggle right
 * press and hold for in
 * press and hold for out
 * */
public class IntakeComponent
{
    private final Talon motorLeft;
    private final Talon motorRight;

    private final DoubleSolenoid intakeDoubleSolenoidLeft;
    private final DoubleSolenoid intakeDoubleSolenoidRight;

    public IntakeComponent()
    {
        this.motorLeft = new Talon(ElectronicsConstants.INTAKE_LEFT_TALON_CHANNEL);
        this.motorRight = new Talon(ElectronicsConstants.INTAKE_RIGHT_TALON_CHANNEL);

        this.intakeDoubleSolenoidLeft = new DoubleSolenoid(
            ElectronicsConstants.INTAKE_LEFT_ARM_CHANNEL_A,
            ElectronicsConstants.INTAKE_LEFT_ARM_CHANNEL_B);

        this.intakeDoubleSolenoidRight = new DoubleSolenoid(
            ElectronicsConstants.INTAKE_RIGHT_ARM_CHANNEL_A,
            ElectronicsConstants.INTAKE_RIGHT_ARM_CHANNEL_B);
    }

    /**
     * Method to move the intake
    */
    public void setIntake(boolean shouldForward)
    {
        if (shouldForward)
        {
            this.intakeDoubleSolenoidLeft.set(Value.kForward);
            this.intakeDoubleSolenoidRight.set(Value.kForward);
        }
        else
        {
            this.intakeDoubleSolenoidLeft.set(Value.kReverse);
            this.intakeDoubleSolenoidRight.set(Value.kReverse);
        }
    }

    /**
     * Method left intake solenoid toggle
    */
    public void toggleLeftIntake()
    {
        this.intakeDoubleSolenoidLeft.set(!this.intakeDoubleSolenoidLeft.get());
        //SmartDashboardLogger.
    }

    /**
     * Method right intake solenoid toggle
    */
    public void toggleRightIntake()
    {
        this.intakeDoubleSolenoidRight.set(!this.intakeDoubleSolenoidRight.get());
    }

    /**
     * sets the intake motors speed
     * @param velocity to set the motors to,
     * values < 0 are inwards, values > 0 forwards, 0 motor off
    */
    public void setIntakeMotorSpeed(double velocity)
    {
        this.motorLeft.set(velocity);
        this.motorRight.set(velocity);
    }
}
