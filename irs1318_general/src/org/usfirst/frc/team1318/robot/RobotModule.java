package org.usfirst.frc.team1318.robot;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Named;
import javax.inject.Singleton;

import org.usfirst.frc.team1318.robot.common.IController;
import org.usfirst.frc.team1318.robot.common.IDashboardLogger;
import org.usfirst.frc.team1318.robot.common.SmartDashboardLogger;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.CompressorWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.DigitalInputWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.EncoderWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.ICompressor;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IDigitalInput;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IEncoder;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IJoystick;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IMotor;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.IPowerDistributionPanel;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.ISolenoid;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.ITimer;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.JoystickWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.PowerDistributionPanelWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.SolenoidWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.TimerWrapper;
import org.usfirst.frc.team1318.robot.common.wpilibmocks.VictorWrapper;
import org.usfirst.frc.team1318.robot.compressor.CompressorController;
import org.usfirst.frc.team1318.robot.driver.ButtonMap;
import org.usfirst.frc.team1318.robot.driver.IButtonMap;
import org.usfirst.frc.team1318.robot.drivetrain.DriveTrainController;
import org.usfirst.frc.team1318.robot.general.PositionManager;
import org.usfirst.frc.team1318.robot.general.PowerManager;
import org.usfirst.frc.team1318.robot.vision.VisionManager;

import com.google.inject.AbstractModule;
import com.google.inject.Injector;
import com.google.inject.Provides;

public class RobotModule extends AbstractModule
{
    @Override
    protected void configure()
    {
    }

    @Singleton
    @Provides
    public IDashboardLogger getLogger()
    {
        IDashboardLogger logger = new SmartDashboardLogger();
        //        try
        //        {
        //            String fileName = String.format("/home/lvuser/%1$d.csv", Calendar.getInstance().getTime().getTime());
        //            IDashboardLogger csvLogger = new CSVLogger(fileName, new String[] { "r.time", "vision.mAngle", "vision.dist" });
        //            logger = new MultiLogger(logger, csvLogger);
        //        }
        //        catch (IOException e)
        //        {
        //            e.printStackTrace();
        //        }

        return logger;
    }

    @Singleton
    @Provides
    public IButtonMap getButtonMap()
    {
        return new ButtonMap();
    }

    @Singleton
    @Provides
    public ITimer getTimer()
    {
        return new TimerWrapper();
    }

    @Singleton
    @Provides
    public ControllerManager getControllerManager(Injector injector)
    {
        List<IController> controllerList = new ArrayList<>();
        controllerList.add(injector.getInstance(PowerManager.class));
        controllerList.add(injector.getInstance(PositionManager.class));
        controllerList.add(injector.getInstance(VisionManager.class));
        controllerList.add(injector.getInstance(CompressorController.class));
        controllerList.add(injector.getInstance(DriveTrainController.class));
        return new ControllerManager(controllerList);
    }

    @Singleton
    @Provides
    @Named("USER_DRIVER_JOYSTICK")
    public IJoystick getDriverJoystick()
    {
        return new JoystickWrapper(ElectronicsConstants.JOYSTICK_DRIVER_PORT);
    }

    @Singleton
    @Provides
    @Named("USER_CODRIVER_JOYSTICK")
    public IJoystick getCoDriverJoystick()
    {
        return new JoystickWrapper(ElectronicsConstants.JOYSTICK_CO_DRIVER_PORT);
    }

    @Singleton
    @Provides
    public ICompressor getCompressor()
    {
        return new CompressorWrapper(ElectronicsConstants.PCM_B_MODULE);
    }

    @Singleton
    @Provides
    public IPowerDistributionPanel getPowerManagerPdp()
    {
        return new PowerDistributionPanelWrapper();
    }

    @Singleton
    @Provides
    @Named("VISION_RING_LIGHT")
    public ISolenoid getVisionRingLight()
    {
        SolenoidWrapper ringLight = new SolenoidWrapper(
            ElectronicsConstants.PCM_B_MODULE,
            ElectronicsConstants.VISION_RING_LIGHT_CHANNEL);

        return ringLight;
    }

    @Singleton
    @Provides
    @Named("DRIVETRAIN_LEFTMOTOR")
    public IMotor getDriveTrainLeftMotor()
    {
        return new VictorWrapper(ElectronicsConstants.DRIVETRAIN_LEFT_TALON_CHANNEL);
    }

    @Singleton
    @Provides
    @Named("DRIVETRAIN_RIGHTMOTOR")
    public IMotor getDriveTrainRightMotor()
    {
        return new VictorWrapper(ElectronicsConstants.DRIVETRAIN_RIGHT_TALON_CHANNEL);
    }

    @Singleton
    @Provides
    @Named("DRIVETRAIN_LEFTENCODER")
    public IEncoder getDriveTrainLeftEncoder()
    {
        EncoderWrapper encoder = new EncoderWrapper(
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_LEFT_ENCODER_CHANNEL_B);

        encoder.setDistancePerPulse(HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE);

        return encoder;
    }

    @Singleton
    @Provides
    @Named("DRIVETRAIN_RIGHTENCODER")
    public IEncoder getDriveTrainRightEncoder()
    {
        EncoderWrapper encoder = new EncoderWrapper(
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A,
            ElectronicsConstants.DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B);

        encoder.setDistancePerPulse(HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE);

        return encoder;
    }

    @Singleton
    @Provides
    @Named("AUTO_DIP_SWITCH_A")
    public IDigitalInput getAutoDipSwitchA()
    {
        return new DigitalInputWrapper(ElectronicsConstants.AUTO_DIP_SWITCH_A_CHANNEL);
    }
}
