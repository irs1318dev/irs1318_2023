package frc.team1318.robot.common.robotprovider;

import java.util.HashMap;
import java.util.Map;

public class FauxbotSensorManager
{
    private static Map<Integer, FauxbotSensorBase> sensorMap = new HashMap<>();
    private static int highestPort = 0;

    public static void set(int port, FauxbotSensorBase sensor)
    {
        if (FauxbotSensorManager.sensorMap.containsKey(port))
        {
            throw new RuntimeException("Don't expect port " + port + " to be specified multiple times!");
        }

        FauxbotSensorManager.sensorMap.put(port, sensor);
        if (FauxbotSensorManager.highestPort < port)
        {
            FauxbotSensorManager.highestPort = port;
        }
    }

    public static FauxbotSensorBase get(int port)
    {
        if (!FauxbotSensorManager.sensorMap.containsKey(port))
        {
            return null;
        }

        return FauxbotSensorManager.sensorMap.get(port);
    }

    public static int getHightestPort()
    {
        return FauxbotSensorManager.highestPort;
    }
}
