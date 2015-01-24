package Arm;

import org.usfirst.frc.team1318.robot.ElectronicsConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ArmComponent
{
    public DoubleSolenoid trombone;
    public DoubleSolenoid moveLinkage;
    public DoubleSolenoid extendLinkage;

    public ArmComponent()
    {
        this.trombone = new DoubleSolenoid(ElectronicsConstants.TROMBONE_SOLANOID_EXTEND, ElectronicsConstants.TROMBONE_SOLANOID_RETRACT);
        this.moveLinkage = new DoubleSolenoid(ElectronicsConstants.MOVE_LINK_SOLANOID_EXTEND,
            ElectronicsConstants.MOVE_LINK_SOLANOID_RETRACT);
        this.extendLinkage = new DoubleSolenoid(ElectronicsConstants.EXTEND_LINK_SOLANOID_EXTEND,
            ElectronicsConstants.EXTEND_LINK_SOLANOID_RETRACT);
    }

    public void setTromboneSolenoidState(Value state)
    {
        this.trombone.set(state);
    }

    public void setMoveLinkageSolenoidState(Value state)
    {
        this.moveLinkage.set(state);
    }

    public void setExtendLinkageSolenoidState(Value state)
    {
        this.extendLinkage.set(state);
    }
}
