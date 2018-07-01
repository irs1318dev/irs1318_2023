package org.usfirst.frc.team1318.robot;

import org.junit.Test;

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
