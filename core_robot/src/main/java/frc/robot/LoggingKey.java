package frc.robot;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r.state", true),
    RobotTime("r.time", true),
    RobotMatch("r.match"),
    DriverIsAuto("driver.isAuto"),
    DriverActiveMacros("driver.activeMacros", true),
    DriverActiveShifts("driver.activeShifts"),
    AutonomousSelection("auto.selected"),
    DriveTrainLeftVelocity("dt.leftVelocity", true),
    DriveTrainLeftTicks("dt.leftTicks", true),
    DriveTrainLeftError("dt.leftError", true),
    DriveTrainRightVelocity("dt.rightVelocity", true),
    DriveTrainRightTicks("dt.rightTicks", true),
    DriveTrainRightError("dt.rightError", true),
    DriveTrainLeftVelocityGoal("dt.leftVelocityGoal", true),
    DriveTrainRightVelocityGoal("dt.rightVelocityGoal", true),
    DriveTrainLeftPositionGoal("dt.leftPositionGoal", true),
    DriveTrainRightPositionGoal("dt.rightPositionGoal", true),
    OffboardVisionX("rpi.x", true),
    OffboardVisionY("rpi.y", true),
    OffboardVisionDistance("rpi.distance", true),
    OffboardVisionHorizontalAngle("rpi.horizontalAngle", true),
    OffboardVisionEnableVision("rpi.enableVision", true),
    OffboardVisionEnableStream("rpi.enableStream", true),
    OffboardVisionEnableProcessing("rpi.enableProcessing", true),
    PositionOdometryAngle("pos.odom_angle"),
    PositionOdometryX("pos.odom_x"),
    PositionOdometryY("pos.odom_y"),
    PositionNavxConnected("pos.navx_connected", true),
    PositionNavxAngle("pos.navx_angle", true),
    PositionNavxX("pos.navx_x"),
    PositionNavxY("pos.navx_y"),
    PositionNavxZ("pos.navx_z"),
    PositionStartingAngle("pos.startingAngle");

    public final String value;
    public final boolean shouldLog;
    private LoggingKey(String value)
    {
        this(value, false);
    }

    private LoggingKey(String value, boolean shouldLog)
    {
        this.value = value;
        this.shouldLog = shouldLog;
    }
}
