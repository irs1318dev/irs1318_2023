package frc.team1318.robot.common.robotprovider;

import java.util.HashMap;
import java.util.Map;

public class FauxbotActuatorManager
{
    private static Map<Integer, FauxbotActuatorBase> actuatorMap = new HashMap<>();
    private static int highestPort = 0;

    public static void set(int port, FauxbotActuatorBase sensor)
    {
        FauxbotActuatorManager.actuatorMap.put(port, sensor);
        if (FauxbotActuatorManager.highestPort < port)
        {
            FauxbotActuatorManager.highestPort = port;
        }
    }

    public static FauxbotActuatorBase get(int port)
    {
        if (!FauxbotActuatorManager.actuatorMap.containsKey(port))
        {
            return null;
        }

        return FauxbotActuatorManager.actuatorMap.get(port);
    }

    public static int getHightestPort()
    {
        return FauxbotActuatorManager.highestPort;
    }
}
