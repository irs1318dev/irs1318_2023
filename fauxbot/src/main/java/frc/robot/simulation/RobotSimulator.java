package frc.robot.simulation;

import java.util.HashMap;
import java.util.Map;

import frc.robot.IRealWorldSimulator;
import frc.robot.common.robotprovider.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import javafx.scene.canvas.Canvas;

@Singleton
public class RobotSimulator implements IRealWorldSimulator
{
    private static final FauxbotActuatorConnection DriveLeftMasterChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 1);
    private static final FauxbotActuatorConnection DriveLeftFollowerChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 2);
    private static final FauxbotActuatorConnection DriveRightMasterChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 3);
    private static final FauxbotActuatorConnection DriveRightFollowerChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, 4);

    private static final FauxbotSensorConnection DriveLeftEncoderChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 1);
    private static final FauxbotSensorConnection DriveRightEncoderChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, 3);

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
            this.put(RobotSimulator.DriveLeftMasterChannel, "DriveTrain Left motor (master)");
            this.put(RobotSimulator.DriveLeftFollowerChannel, "DriveTrain Left motor (follower)");
            this.put(RobotSimulator.DriveRightMasterChannel, "DriveTrain Right motor (master)");
            this.put(RobotSimulator.DriveRightFollowerChannel, "DriveTrain Right motor (follower)");
        }
    };

    @Inject
    public RobotSimulator()
    {
    }

    public String getSensorName(FauxbotSensorConnection connection)
    {
        if (this.sensorNameMap.containsKey(connection))
        {
            return this.sensorNameMap.get(connection);
        }

        return "Sensor " + connection;
    }

    public double getSensorMin(FauxbotSensorConnection connection)
    {
        if (connection.equals(RobotSimulator.DriveLeftEncoderChannel) ||
            connection.equals(RobotSimulator.DriveRightEncoderChannel))
        {
            return -5000.0;
        }

        return 0;
    }

    public double getSensorMax(FauxbotSensorConnection connection)
    {
        if (connection.equals(RobotSimulator.DriveLeftEncoderChannel) ||
            connection.equals(RobotSimulator.DriveRightEncoderChannel))
        {
            return 5000.0;
        }

        return 0;
    }

    public String getActuatorName(FauxbotActuatorConnection connection)
    {
        if (this.motorNameMap.containsKey(connection))
        {
            return this.motorNameMap.get(connection);
        }

        return "Motor " + connection;
    }

    public double getMotorMin(FauxbotActuatorConnection connection)
    {
        if (connection.equals(RobotSimulator.DriveLeftMasterChannel) ||
            connection.equals(RobotSimulator.DriveLeftFollowerChannel) ||
            connection.equals(RobotSimulator.DriveRightMasterChannel) ||
            connection.equals(RobotSimulator.DriveRightFollowerChannel))
        {
            return -1600.0;
        }

        return -1.0;
    }

    public double getMotorMax(FauxbotActuatorConnection connection)
    {
        if (connection.equals(RobotSimulator.DriveLeftMasterChannel) ||
            connection.equals(RobotSimulator.DriveLeftFollowerChannel) ||
            connection.equals(RobotSimulator.DriveRightMasterChannel) ||
            connection.equals(RobotSimulator.DriveRightFollowerChannel))
        {
            return 1600.0;
        }

        return 1.0;
    }

    public void update()
    {
    }

    public void draw(Canvas canvas)
    {
    }
}
