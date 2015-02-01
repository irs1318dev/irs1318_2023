package org.usfirst.frc.team1318.robot.Intake;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

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
    public static final String INTAKE_MOTOR_SPEED = "i.motorSpeed";
    public static final String LEFT_INTAKE_ARM_DIRECTION = "i.leftArmDirection";
    public static final String RIGHT_INTAKE_ARM_DIRECTION = "i.rightArmDirection";

    private boolean solenoidLeftState;
    private boolean solenoidRightState;
    private double intakeMotorVelocity;

    public IntakeComponent()
    {
        this.leftTalon = new Talon(ElectronicsConstants.INTAKE_LEFT_TALON_CHANNEL);
        this.rightTalon = new Talon(ElectronicsConstants.INTAKE_RIGHT_TALON_CHANNEL);

        this.intakeDoubleSolenoidLeft = new DoubleSolenoid(
            ElectronicsConstants.INTAKE_LEFT_ARM_CHANNEL_A,
            ElectronicsConstants.INTAKE_LEFT_ARM_CHANNEL_B);

        this.intakeDoubleSolenoidRight = new DoubleSolenoid(
            ElectronicsConstants.INTAKE_RIGHT_ARM_CHANNEL_A,
            ElectronicsConstants.INTAKE_RIGHT_ARM_CHANNEL_B);
    }

    /**
     * sets both solenoids on the intake to extend or retract
     * @param shouldForward if true, both pistons will extend if false both will retract
    */

    public void setIntake(boolean leftForward, boolean rightForward)
    {
        if (leftForward)
        {
            this.intakeDoubleSolenoidLeft.set(Value.kForward);
            solenoidLeftState = true;
            SmartDashboardLogger.putBoolean(LEFT_INTAKE_ARM_DIRECTION, solenoidLeftState);
        }
        else
        {
            this.intakeDoubleSolenoidLeft.set(Value.kReverse);
            solenoidLeftState = false;
            SmartDashboardLogger.putBoolean(LEFT_INTAKE_ARM_DIRECTION, solenoidLeftState);
        }

        if (rightForward)
        {
            this.intakeDoubleSolenoidRight.set(Value.kForward);
            solenoidRightState = true;
            SmartDashboardLogger.putBoolean(RIGHT_INTAKE_ARM_DIRECTION, solenoidRightState);
        }
        else
        {
            this.intakeDoubleSolenoidRight.set(Value.kReverse);
            solenoidRightState = false;
            SmartDashboardLogger.putBoolean(RIGHT_INTAKE_ARM_DIRECTION, solenoidRightState);
        }
    }

    /**
     * sets the intake motors speed
     * @param velocity to set the motors to,
     * values < 0 are inwards, values > 0 forwards, 0 motor off
    */
    public void setIntakeMotorSpeed(double velocity)
    {
        this.intakeMotorVelocity = velocity;
        this.leftTalon.set(intakeMotorVelocity);
        this.rightTalon.set(intakeMotorVelocity);
        SmartDashboardLogger.putNumber(INTAKE_MOTOR_SPEED, intakeMotorVelocity);
    }
}
