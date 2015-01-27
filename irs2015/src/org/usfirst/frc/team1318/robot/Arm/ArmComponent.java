package org.usfirst.frc.team1318.robot.Arm;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ArmComponent
{
    public DoubleSolenoid trombone;
    public DoubleSolenoid moveLinkage;
    public DoubleSolenoid extendLinkage;

    //logging constants
    private static final String TROMBONE_STATE_LOG_KEY = "dt.TromboneState";
    private static final String MOVE_LINKAGE_STATE_LOG_KEY = "dt.MoveLinkageState";
    private static final String EXTEND_LINKAGE_STATE_LOG_KEY = "dt.ExtendLinkageState";

    //constructor requires constants for extension and retraction of 3 solenoids that make up the arm 
    public ArmComponent()
    {
        this.trombone = new DoubleSolenoid(ElectronicsConstants.TROMBONE_SOLANOID_EXTEND, ElectronicsConstants.TROMBONE_SOLANOID_RETRACT);
        this.moveLinkage = new DoubleSolenoid(ElectronicsConstants.MOVE_LINK_SOLANOID_EXTEND,
            ElectronicsConstants.MOVE_LINK_SOLANOID_RETRACT);
        this.extendLinkage = new DoubleSolenoid(ElectronicsConstants.EXTEND_LINK_SOLANOID_EXTEND,
            ElectronicsConstants.EXTEND_LINK_SOLANOID_RETRACT);
    }

    /**
     * @param state to which to set the Trombone Solenoid
     */
    public void setTromboneSolenoidState(Value state)
    {
        this.trombone.set(state);
        SmartDashboardLogger.putNumber(ArmComponent.TROMBONE_STATE_LOG_KEY, trombone.get().value);

    }

    /**
     * @param state to which to set the MoveLinkage Solenoid
     */
    public void setMoveLinkageSolenoidState(Value state)
    {
        this.moveLinkage.set(state);
        SmartDashboardLogger.putNumber(ArmComponent.MOVE_LINKAGE_STATE_LOG_KEY, moveLinkage.get().value);
    }

    /**
     * @param state to which to set the ExtendLinkage Solenoid
     */
    public void setExtendLinkageSolenoidState(Value state)
    {
        this.extendLinkage.set(state);
        SmartDashboardLogger.putNumber(ArmComponent.EXTEND_LINKAGE_STATE_LOG_KEY, extendLinkage.get().value);
    }

}
