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
    private final Talon motorLeft;
    private final Talon motorRight;

    private final DoubleSolenoid intakeDoubleSolenoidLeft;
    private final DoubleSolenoid intakeDoubleSolenoidRight;

    // SmartDashboard keys
    public static final String INTAKEMOTORSPEED = "i.motorspeed";
    public static final String LEFTINTAKEARMDIRECTION = "i.leftarmdirection";
    public static final String RIGHTINTAKEARMDIRECTION = "i.rightarmdirection";

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
     * sets both solenoids on the intake to extend or retract
     * @param shouldForward if true, both pistons will extend if false both will retract
    */
    public void setIntake(boolean shouldForward)
    {
        if (shouldForward)
        {
            this.intakeDoubleSolenoidLeft.set(Value.kForward);
            this.intakeDoubleSolenoidRight.set(Value.kForward);
            SmartDashboardLogger.putString(LEFTINTAKEARMDIRECTION, intakeDoubleSolenoidLeft.get().toString());
            SmartDashboardLogger.putString(RIGHTINTAKEARMDIRECTION, intakeDoubleSolenoidRight.get().toString());
        }
        else
        {
            this.intakeDoubleSolenoidLeft.set(Value.kReverse);
            this.intakeDoubleSolenoidRight.set(Value.kReverse);
            SmartDashboardLogger.putString(LEFTINTAKEARMDIRECTION, intakeDoubleSolenoidLeft.get().toString());
            SmartDashboardLogger.putString(RIGHTINTAKEARMDIRECTION, intakeDoubleSolenoidRight.get().toString());
        }
    }

    /**
     * Toggles the left intake solenoid
     * If the piston is extending, it will be switched to retracting
     * If the piston is off or retracting, it will be switched to extending
    */
    public void toggleLeftIntake()
    {
        this.intakeDoubleSolenoidLeft.set(
            intakeDoubleSolenoidLeft.get() == DoubleSolenoid.Value.kForward ?
                DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        // not sure if DoubleSolenoidValue.toString() returns an understandable result
        SmartDashboardLogger.putString(LEFTINTAKEARMDIRECTION, intakeDoubleSolenoidLeft.get().toString());
    }

    /**
     * Toggles the right intake solenoid
     * If the piston is extending, it will be switched to retracting
     * If the piston is off or retracting, it will be switched to extending
    */
    public void toggleRightIntake()
    {
        this.intakeDoubleSolenoidRight.set(
            intakeDoubleSolenoidRight.get() == DoubleSolenoid.Value.kForward ?
                DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        // not sure if DoubleSolenoidValue.toString() returns an understandable result
        SmartDashboardLogger.putString(RIGHTINTAKEARMDIRECTION, intakeDoubleSolenoidRight.get().toString());
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
        SmartDashboardLogger.putNumber(INTAKEMOTORSPEED, motorLeft.get());
    }

    // stops both solenoids, leaving the pistons unmoving
    public void stopSolenoids()
    {
        intakeDoubleSolenoidLeft.set(Value.kOff);
        intakeDoubleSolenoidRight.set(Value.kOff);
        SmartDashboardLogger.putString(LEFTINTAKEARMDIRECTION, intakeDoubleSolenoidLeft.get().toString());
        SmartDashboardLogger.putString(RIGHTINTAKEARMDIRECTION, intakeDoubleSolenoidRight.get().toString());
    }
}
