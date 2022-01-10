package frc.robot.common.robotprovider;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class TalonSRXWrapper implements ITalonSRX
{
    private static final int pidIdx = 0;
    private static final int timeoutMS = 10;

    final TalonSRX wrappedObject;

    private ControlMode controlMode;

    public TalonSRXWrapper(int deviceNumber)
    {
        this.wrappedObject = new TalonSRX(deviceNumber);
        this.controlMode = ControlMode.PercentOutput;
    }

    public void set(double value)
    {
        this.wrappedObject.set(this.controlMode, value);
    }

    public void follow(ITalonSRX talonSRX)
    {
        this.wrappedObject.follow(((TalonSRXWrapper)talonSRX).wrappedObject);
    }

    public void follow(ITalonFX talonFX)
    {
        this.wrappedObject.follow(((TalonFXWrapper)talonFX).wrappedObject);
    }

    public void follow(IVictorSPX victorSPX)
    {
        this.wrappedObject.follow(((VictorSPXWrapper)victorSPX).wrappedObject);
    }

    public void setControlMode(TalonXControlMode mode)
    {
        if (mode == TalonXControlMode.PercentOutput)
        {
            this.controlMode = ControlMode.PercentOutput;
        }
        else if (mode == TalonXControlMode.Disabled)
        {
            this.controlMode = ControlMode.Disabled;
        }
        else if (mode == TalonXControlMode.Follower)
        {
            this.controlMode = ControlMode.Follower;
        }
        else if (mode == TalonXControlMode.Position)
        {
            this.controlMode = ControlMode.Position;
        }
        else if (mode == TalonXControlMode.MotionMagicPosition)
        {
            this.controlMode = ControlMode.MotionMagic;
        }
        else if (mode == TalonXControlMode.Velocity)
        {
            this.controlMode = ControlMode.Velocity;
        }
    }

    public void setSensorType(TalonXFeedbackDevice feedbackDevice)
    {
        FeedbackDevice device;
        if (feedbackDevice == TalonXFeedbackDevice.QuadEncoder)
        {
            device = FeedbackDevice.QuadEncoder;
        }
        else if (feedbackDevice == TalonXFeedbackDevice.PulseWidthEncodedPosition)
        {
            device = FeedbackDevice.PulseWidthEncodedPosition;
        }
        else
        {
            return;
        }

        this.wrappedObject.configSelectedFeedbackSensor(device, TalonSRXWrapper.pidIdx, 0);
    }

    public void setPIDFFramePeriod(int periodMS)
    {
        this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, periodMS, TalonSRXWrapper.timeoutMS);
    }

    public void setFeedbackFramePeriod(int periodMS)
    {
        this.wrappedObject.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, periodMS, TalonSRXWrapper.timeoutMS);
    }

    public void configureVelocityMeasurements(int periodMS, int windowSize)
    {
        this.wrappedObject.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.valueOf(periodMS), TalonSRXWrapper.timeoutMS);
        this.wrappedObject.configVelocityMeasurementWindow(windowSize, TalonSRXWrapper.timeoutMS);
    }

    public void configureAllowableClosedloopError(int slotId, int error)
    {
        this.wrappedObject.configAllowableClosedloopError(slotId, error, TalonSRXWrapper.timeoutMS);
    }

    public void setSelectedSlot(int slotId)
    {
        this.wrappedObject.selectProfileSlot(slotId, TalonSRXWrapper.pidIdx);
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
        this.wrappedObject.config_kP(slotId, p, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_kI(slotId, i, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_kD(slotId, d, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_kF(slotId, f, TalonSRXWrapper.timeoutMS);
    }

    public void setMotionMagicPIDF(double p, double i, double d, double f, int velocity, int acceleration, int slotId)
    {
        this.wrappedObject.config_kP(slotId, p, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_kI(slotId, i, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_kD(slotId, d, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_kF(slotId, f, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.configMotionCruiseVelocity(velocity, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.configMotionAcceleration(acceleration, TalonSRXWrapper.timeoutMS);
    }

    public void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int slotId)
    {
        this.wrappedObject.config_kP(slotId, p, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_kI(slotId, i, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_kD(slotId, d, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_kF(slotId, f, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.config_IntegralZone(slotId, izone, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.configClosedloopRamp(closeLoopRampRate, TalonSRXWrapper.timeoutMS);
    }

    public void setForwardLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        LimitSwitchSource source = LimitSwitchSource.Deactivated;
        if (enabled)
        {
            source = LimitSwitchSource.FeedbackConnector;
        }

        LimitSwitchNormal type = LimitSwitchNormal.NormallyClosed;
        if (normallyOpen)
        {
            type = LimitSwitchNormal.NormallyOpen;
        }

        this.wrappedObject.configForwardLimitSwitchSource(
            source,
            type,
            TalonSRXWrapper.timeoutMS);
    }

    public void setReverseLimitSwitch(boolean enabled, boolean normallyOpen)
    {
        LimitSwitchSource source = LimitSwitchSource.Deactivated;
        if (enabled)
        {
            source = LimitSwitchSource.FeedbackConnector;
        }

        LimitSwitchNormal type = LimitSwitchNormal.NormallyClosed;
        if (normallyOpen)
        {
            type = LimitSwitchNormal.NormallyOpen;
        }

        this.wrappedObject.configReverseLimitSwitchSource(
            source,
            type,
            TalonSRXWrapper.timeoutMS);
    }

    public void setInvertOutput(boolean invert)
    {
        this.wrappedObject.setInverted(invert);
    }

    public void setInvertSensor(boolean invert)
    {
        this.wrappedObject.setSensorPhase(invert);
    }

    public void setNeutralMode(MotorNeutralMode neutralMode)
    {
        NeutralMode mode;
        if (neutralMode == MotorNeutralMode.Brake)
        {
            mode = NeutralMode.Brake;
        }
        else
        {
            mode = NeutralMode.Coast;
        }

        this.wrappedObject.setNeutralMode(mode);
    }

    public void setVoltageCompensation(boolean enabled, double maxVoltage)
    {
        this.wrappedObject.configVoltageCompSaturation(maxVoltage, TalonSRXWrapper.timeoutMS);
        this.wrappedObject.enableVoltageCompensation(enabled);
    }

    public void stop()
    {
        this.wrappedObject.set(ControlMode.Disabled, 0.0);
    }

    public void setPosition(double position)
    {
        this.wrappedObject.setSelectedSensorPosition(position, TalonSRXWrapper.pidIdx, TalonSRXWrapper.timeoutMS);
    }

    public void reset()
    {
        this.wrappedObject.setSelectedSensorPosition(0.0, TalonSRXWrapper.pidIdx, TalonSRXWrapper.timeoutMS);
    }

    public double getPosition()
    {
        return this.wrappedObject.getSelectedSensorPosition(TalonSRXWrapper.pidIdx);
    }

    public double getVelocity()
    {
        return this.wrappedObject.getSelectedSensorVelocity(TalonSRXWrapper.pidIdx);
    }

    public double getError()
    {
        return this.wrappedObject.getClosedLoopError(TalonSRXWrapper.pidIdx);
    }

    public TalonXLimitSwitchStatus getLimitSwitchStatus()
    {
        SensorCollection collection = this.wrappedObject.getSensorCollection();

        return new TalonXLimitSwitchStatus(
            collection.isFwdLimitSwitchClosed(),
            collection.isRevLimitSwitchClosed());
    }
}
