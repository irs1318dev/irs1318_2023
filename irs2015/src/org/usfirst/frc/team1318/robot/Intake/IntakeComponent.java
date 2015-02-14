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
    private final Talon leftTalon;
    private final Talon rightTalon;

    private final DoubleSolenoid intakeDoubleSolenoidLeft;
    private final DoubleSolenoid intakeDoubleSolenoidRight;

    // SmartDashboard keys
    public static final String INTAKE_MOTOR_SPEED_KEY = "i.motorSpeed";
    public static final String LEFT_INTAKE_ARM_DIRECTION_KEY = "i.leftArmDirection";
    public static final String RIGHT_INTAKE_ARM_DIRECTION_KEY = "i.rightArmDirection";

    public IntakeComponent()
    {
        this.leftTalon = new Talon(ElectronicsConstants.INTAKE_LEFT_TALON_CHANNEL);
        this.rightTalon = new Talon(ElectronicsConstants.INTAKE_RIGHT_TALON_CHANNEL);

        this.intakeDoubleSolenoidLeft = new DoubleSolenoid(
            ElectronicsConstants.PCM_B_MODULE,
            ElectronicsConstants.INTAKE_LEFT_ARM_CHANNEL_A,
            ElectronicsConstants.INTAKE_LEFT_ARM_CHANNEL_B);

        this.intakeDoubleSolenoidRight = new DoubleSolenoid(
            ElectronicsConstants.PCM_B_MODULE,
            ElectronicsConstants.INTAKE_RIGHT_ARM_CHANNEL_A,
            ElectronicsConstants.INTAKE_RIGHT_ARM_CHANNEL_B);
    }

    /**
     * sets both solenoids on the intake to extend or retract
     * @param leftForward if true the left piston will extend, 
     * if false the left piston will retract
    */

    public void setLeftIntake(boolean leftForward)
    {
        if (leftForward)
        {
            this.intakeDoubleSolenoidLeft.set(Value.kForward);
        }
        else
        {
            this.intakeDoubleSolenoidLeft.set(Value.kReverse);
        }

        //        SmartDashboardLogger.putBoolean(IntakeComponent.LEFT_INTAKE_ARM_DIRECTION_KEY, leftForward);
    }

    /**
     * sets both solenoids on the intake to extend or retract
     * @param rightForward if true the right piston will extend, 
     * if false the right piston will retract
    */

    public void setRightIntake(boolean rightForward)
    {
        if (rightForward)
        {
            this.intakeDoubleSolenoidRight.set(Value.kForward);
        }
        else
        {
            this.intakeDoubleSolenoidRight.set(Value.kReverse);
        }

        //        SmartDashboardLogger.putBoolean(IntakeComponent.RIGHT_INTAKE_ARM_DIRECTION_KEY, rightForward);
    }

    /**
     * sets the intake motors speed
     * @param double velocity to set the motors to,
     * values < 0 are inwards, values > 0 forwards, 0 motor off
     * range [-1,1]
    */
    public void setIntakeMotorSpeed(double velocity)
    {
        this.leftTalon.set(-velocity);
        this.rightTalon.set(velocity);
        //        SmartDashboardLogger.putNumber(IntakeComponent.INTAKE_MOTOR_SPEED_KEY, velocity);
    }
}
