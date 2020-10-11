package frc.robot.simulation;

import java.util.Calendar;
import java.util.HashMap;
import java.util.Map;

import frc.robot.ElectronicsConstants;
import frc.robot.IRealWorldSimulator;
import frc.robot.common.robotprovider.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import javafx.scene.canvas.Canvas;

@Singleton
public class RobotSimulator implements IRealWorldSimulator
{
    private static final FauxbotActuatorConnection DriveLeftPrimaryChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_LEFT_PRIMARY_CAN_ID);
    private static final FauxbotActuatorConnection DriveLeftFollowerChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER_CAN_ID);
    private static final FauxbotActuatorConnection DriveRightPrimaryChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_RIGHT_PRIMARY_CAN_ID);
    private static final FauxbotActuatorConnection DriveRightFollowerChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID);

    private static final FauxbotSensorConnection DriveLeftEncoderChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, RobotSimulator.DriveLeftPrimaryChannel.getPort());
    private static final FauxbotSensorConnection DriveRightEncoderChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, RobotSimulator.DriveRightPrimaryChannel.getPort());

    private final FauxbotSensorConnection[] sensors =
        new FauxbotSensorConnection[]
        {
            RobotSimulator.DriveLeftEncoderChannel,
            RobotSimulator.DriveRightEncoderChannel,
        };

    private final FauxbotActuatorConnection[] actuators =
        new FauxbotActuatorConnection[]
        {
            RobotSimulator.DriveLeftPrimaryChannel,
            RobotSimulator.DriveLeftFollowerChannel,
            RobotSimulator.DriveRightPrimaryChannel,
            RobotSimulator.DriveRightFollowerChannel,
        };

    @SuppressWarnings("serial")
    private final Map<FauxbotSensorConnection, String> sensorNameMap = new HashMap<FauxbotSensorConnection, String>()
    {
        {
            this.put(RobotSimulator.DriveLeftEncoderChannel, "DriveTrain Left encoder");
            this.put(RobotSimulator.DriveRightEncoderChannel, "DriveTrain Right encoder");
        }
    };

    @SuppressWarnings("serial")
    private final Map<FauxbotActuatorConnection, String> motorNameMap = new HashMap<FauxbotActuatorConnection, String>()
    {
        {
            this.put(RobotSimulator.DriveLeftPrimaryChannel, "DriveTrain Left motor (primary)");
            this.put(RobotSimulator.DriveLeftFollowerChannel, "DriveTrain Left motor (follower)");
            this.put(RobotSimulator.DriveRightPrimaryChannel, "DriveTrain Right motor (primary)");
            this.put(RobotSimulator.DriveRightFollowerChannel, "DriveTrain Right motor (follower)");
        }
    };

    @Inject
    public RobotSimulator()
    {
    }

    @Override
    public FauxbotSensorConnection[] getSensors()
    {
        return this.sensors;
    }

    @Override
    public FauxbotActuatorConnection[] getActuators()
    {
        return this.actuators;
    }

    @Override
    public boolean getSensorTextBox(FauxbotSensorConnection connection)
    {
        return false;
    }

    @Override
    public String getSensorName(FauxbotSensorConnection connection)
    {
        if (this.sensorNameMap.containsKey(connection))
        {
            return this.sensorNameMap.get(connection);
        }

        return "Sensor " + connection;
    }

    @Override
    public double getSensorMin(FauxbotSensorConnection connection)
    {
        if (connection.equals(RobotSimulator.DriveLeftEncoderChannel) ||
            connection.equals(RobotSimulator.DriveRightEncoderChannel))
        {
            return -5000.0;
        }

        return 0;
    }

    @Override
    public double getSensorMax(FauxbotSensorConnection connection)
    {
        if (connection.equals(RobotSimulator.DriveLeftEncoderChannel) ||
            connection.equals(RobotSimulator.DriveRightEncoderChannel))
        {
            return 5000.0;
        }

        return 0;
    }

    @Override
    public String getActuatorName(FauxbotActuatorConnection connection)
    {
        if (this.motorNameMap.containsKey(connection))
        {
            return this.motorNameMap.get(connection);
        }

        return "Motor " + connection;
    }

    @Override
    public double getMotorMin(FauxbotActuatorConnection connection)
    {
        if (connection.equals(RobotSimulator.DriveLeftPrimaryChannel) ||
            connection.equals(RobotSimulator.DriveLeftFollowerChannel) ||
            connection.equals(RobotSimulator.DriveRightPrimaryChannel) ||
            connection.equals(RobotSimulator.DriveRightFollowerChannel))
        {
            return -1600.0;
        }

        return -1.0;
    }

    @Override
    public double getMotorMax(FauxbotActuatorConnection connection)
    {
        if (connection.equals(RobotSimulator.DriveLeftPrimaryChannel) ||
            connection.equals(RobotSimulator.DriveLeftFollowerChannel) ||
            connection.equals(RobotSimulator.DriveRightPrimaryChannel) ||
            connection.equals(RobotSimulator.DriveRightFollowerChannel))
        {
            return 1600.0;
        }

        return 1.0;
    }

    @Override
    public boolean shouldSimulatePID()
    {
        return false;
    }

    @Override
    public void update()
    {
    }

    @Override
    public void draw(Canvas canvas)
    {
    }
}
