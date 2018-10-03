package frc.team1318.robot;

import org.junit.jupiter.api.Test;

import com.google.inject.Guice;

public class RobotModuleTest
{
    /**
     * Make sure the wiring is in place.
     */
    @Test
    public void testRobotModule()
    {
        Guice.createInjector(new RobotModule());
    }
}
