package frc.team1318.robot;

import java.util.ArrayList;
import java.util.List;

import javax.inject.Singleton;

import frc.team1318.robot.common.IMechanism;
import frc.team1318.robot.common.MechanismManager;
import frc.team1318.robot.common.robotprovider.IDashboardLogger;
import frc.team1318.robot.common.robotprovider.IRobotProvider;
import frc.team1318.robot.common.robotprovider.ITimer;
import frc.team1318.robot.common.robotprovider.RobotProvider;
import frc.team1318.robot.common.robotprovider.SmartDashboardLogger;
import frc.team1318.robot.common.robotprovider.TimerWrapper;
import frc.team1318.robot.driver.ButtonMap;
import frc.team1318.robot.driver.common.IButtonMap;
import frc.team1318.robot.mechanisms.DriveTrainMechanism;

import com.google.inject.AbstractModule;
import com.google.inject.Injector;
import com.google.inject.Provides;

public class RobotModule extends AbstractModule
{
    @Override
    protected void configure()
    {
        this.bind(IRobotProvider.class).to(RobotProvider.class);
        this.bind(ITimer.class).to(TimerWrapper.class);
        this.bind(IButtonMap.class).to(ButtonMap.class);
    }

    @Singleton
    @Provides
    public MechanismManager getMechanismManager(Injector injector)
    {
        List<IMechanism> mechanismList = new ArrayList<>();
        mechanismList.add(injector.getInstance(DriveTrainMechanism.class));
        //mechanismList.add(injector.getInstance(PositionManager.class));
        //mechanismList.add(injector.getInstance(PowerManager.class));
        //mechanismList.add(injector.getInstance(VisionManager.class));
        //mechanismList.add(injector.getInstance(CompressorMechanism.class));
        //mechanismList.add(injector.getInstance(SomeMechanism.class));
        return new MechanismManager(mechanismList);
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
}
