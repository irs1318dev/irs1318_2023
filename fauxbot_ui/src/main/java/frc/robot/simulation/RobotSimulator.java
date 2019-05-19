package frc.robot.simulation;

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
    private static final FauxbotActuatorConnection DriveLeftMasterChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_LEFT_MASTER_CAN_ID);
    private static final FauxbotActuatorConnection DriveLeftFollower1Channel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER1_CAN_ID);
    private static final FauxbotActuatorConnection DriveLeftFollower2Channel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_LEFT_FOLLOWER2_CAN_ID);
    private static final FauxbotActuatorConnection DriveRightMasterChannel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_RIGHT_MASTER_CAN_ID);
    private static final FauxbotActuatorConnection DriveRightFollower1Channel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER1_CAN_ID);
    private static final FauxbotActuatorConnection DriveRightFollower2Channel = new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, ElectronicsConstants.DRIVETRAIN_RIGHT_FOLLOWER2_CAN_ID);

    private static final FauxbotSensorConnection DriveLeftEncoderChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, RobotSimulator.DriveLeftMasterChannel.getPort());
    private static final FauxbotSensorConnection DriveRightEncoderChannel = new FauxbotSensorConnection(FauxbotSensorConnection.SensorConnector.CAN, RobotSimulator.DriveRightMasterChannel.getPort());

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
            this.put(RobotSimulator.DriveLeftFollower1Channel, "DriveTrain Left motor (follower 1)");
            this.put(RobotSimulator.DriveLeftFollower2Channel, "DriveTrain Left motor (follower 2)");
            this.put(RobotSimulator.DriveRightMasterChannel, "DriveTrain Right motor (master)");
            this.put(RobotSimulator.DriveRightFollower1Channel, "DriveTrain Right motor (follower 1)");
            this.put(RobotSimulator.DriveRightFollower2Channel, "DriveTrain Right motor (follower 2)");
        }
    };

    @Inject
    public RobotSimulator()
    {
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
        if (connection.equals(RobotSimulator.DriveLeftMasterChannel) ||
            connection.equals(RobotSimulator.DriveLeftFollower1Channel) ||
            connection.equals(RobotSimulator.DriveLeftFollower2Channel) ||
            connection.equals(RobotSimulator.DriveRightMasterChannel) ||
            connection.equals(RobotSimulator.DriveRightFollower1Channel) ||
            connection.equals(RobotSimulator.DriveRightFollower2Channel))
        {
            return -1600.0;
        }

        return -1.0;
    }

    @Override
    public double getMotorMax(FauxbotActuatorConnection connection)
    {
        if (connection.equals(RobotSimulator.DriveLeftMasterChannel) ||
            connection.equals(RobotSimulator.DriveLeftFollower1Channel) ||
            connection.equals(RobotSimulator.DriveLeftFollower2Channel) ||
            connection.equals(RobotSimulator.DriveRightMasterChannel) ||
            connection.equals(RobotSimulator.DriveRightFollower1Channel) ||
            connection.equals(RobotSimulator.DriveRightFollower2Channel))
        {
            return 1600.0;
        }

        return 1.0;
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
