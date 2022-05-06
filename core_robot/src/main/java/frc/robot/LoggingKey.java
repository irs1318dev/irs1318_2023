package frc.robot;

/**
 * Keys describing logging 
 */
public enum LoggingKey
{
    RobotState("r.state", true),
    RobotTime("r.time", true),
    RobotMatch("r.match"),
    RobotCrash("r.crash", true),
    DriverMode("driver.mode"),
    DriverActiveMacros("driver.activeMacros", true),
    DriverActiveShifts("driver.activeShifts"),
    AutonomousSelection("auto.selected"),
    AutonomousDSMessage("auto.dsMessage"),
    OffboardVisionTargetDistance("rpi.v_distance", true),
    OffboardVisionTargetHorizontalAngle("rpi.v_horizontalAngle", true),
    OffboardVisionGamePieceDistance("rpi.g_distance", true),
    OffboardVisionGamePieceHorizontalAngle("rpi.g_horizontalAngle", true),
    OffboardVisionEnableVision("rpi.enableVision", true),
    OffboardVisionEnableStream("rpi.enableStream", true),
    OffboardVisionEnableProcessing("rpi.processingSetting", true),
    OffboardVisionMissedHeartbeats("rpi.missedHeartbeats", true),
    NavxStartingAngle("navx.startingAngle"),
    PigeonState("pigeon.state"),
    PigeonYaw("pigeon.yaw", true),
    PigeonPitch("pigeon.pitch"),
    PigeonRoll("pigeon.roll"),
    PigeonStartingYaw("pigeon.startingYaw"),
    NavxConnected("navx.isConnected"),
    NavxAngle("navx.angle"),
    NavxPitch("navx.pitch"),
    NavxRoll("navx.roll"),
    NavxYaw("navx.yaw"),
    NavxX("navx.x"),
    NavxY("navx.y"),
    NavxZ("navx.z"),

    DriveTrainDesiredAngle("dt.angle_goal", true),
    DriveTrainAngle("dt.angle", true),
    DriveTrainXPosition("dt.xpos", true),
    DriveTrainYPosition("dt.ypos", true),
    DriveTrainXPositionGoal("dt.xpos_goal", true),
    DriveTrainYPositionGoal("dt.ypos_goal", true),
    DriveTrainFieldOriented("dt.field_oriented", true),
    DriveTrainMaintainOrientation("dt.maintain_orientation", true),

    DriveTrainAbsoluteEncoderAngle1("dt.absenc_ang1"),
    DriveTrainDriveVelocity1("dt.drive_vel1"),
    DriveTrainDrivePosition1("dt.drive_pos1"),
    DriveTrainDriveError1("dt.drive_err1"),
    DriveTrainDriveVelocityGoal1("dt.drive_goal1"),
    DriveTrainSteerVelocity1("dt.steer_vel1"),
    DriveTrainSteerPosition1("dt.steer_pos1"),
    DriveTrainSteerAngle1("dt.steer_ang1"),
    DriveTrainSteerError1("dt.steer_err1"),
    DriveTrainSteerPositionGoal1("dt.steer_goal1"),

    DriveTrainAbsoluteEncoderAngle2("dt.absenc_ang2"),
    DriveTrainDriveVelocity2("dt.drive_vel2"),
    DriveTrainDrivePosition2("dt.drive_pos2"),
    DriveTrainDriveError2("dt.drive_err2"),
    DriveTrainDriveVelocityGoal2("dt.drive_goal2"),
    DriveTrainSteerVelocity2("dt.steer_vel2"),
    DriveTrainSteerPosition2("dt.steer_pos2"),
    DriveTrainSteerAngle2("dt.steer_ang2"),
    DriveTrainSteerError2("dt.steer_err2"),
    DriveTrainSteerPositionGoal2("dt.steer_goal2"),

    DriveTrainAbsoluteEncoderAngle3("dt.absenc_ang3"),
    DriveTrainDriveVelocity3("dt.drive_vel3"),
    DriveTrainDrivePosition3("dt.drive_pos3"),
    DriveTrainDriveError3("dt.drive_err3"),
    DriveTrainDriveVelocityGoal3("dt.drive_goal3"),
    DriveTrainSteerVelocity3("dt.steer_vel3"),
    DriveTrainSteerPosition3("dt.steer_pos3"),
    DriveTrainSteerAngle3("dt.steer_ang3"),
    DriveTrainSteerError3("dt.steer_err3"),
    DriveTrainSteerPositionGoal3("dt.steer_goal3"),

    DriveTrainAbsoluteEncoderAngle4("dt.absenc_ang4"),
    DriveTrainDriveVelocity4("dt.drive_vel4"),
    DriveTrainDrivePosition4("dt.drive_pos4"),
    DriveTrainDriveError4("dt.drive_err4"),
    DriveTrainDriveVelocityGoal4("dt.drive_goal4"),
    DriveTrainSteerVelocity4("dt.steer_vel4"),
    DriveTrainSteerPosition4("dt.steer_pos4"),
    DriveTrainSteerAngle4("dt.steer_ang4"),
    DriveTrainSteerError4("dt.steer_err4"),
    DriveTrainSteerPositionGoal4("dt.steer_goal4"),

    CompressorPreassure("com.pres"),

    ClimberWinchPower("c.winch_pow"),
    ClimberWinchPosition("c.winch_pos"),
    ClimberWinchError("c.winch_error"),
    ClimberWinchRetracted("c.winch_retracted"),
    ClimberWinchDesiredPosition("c.winch_goal"),
    ClimberHookGrasped("c.hook_grasp"),
    ClimberArmOut("c.arm_out"),
    ClimberWinchLockIn("c.lock_in"),

    CargoIntakePower("cg.intake_pow"),
    CargoFlywheelPower("cg.flywheel_pow"),
    CargoFlywheelPosition("cg.flywheel_pos"),
    CargoFlywheelVelocity("cg.flywheel_vel", true),
    CargoFlywheelError("cg.flywheel_err", true),
    CargoFlywheelDesiredVelocity("cg.flywheel_goal", true),
    CargoConveyorState("cg.conveyor"),
    CargoFeederSensor("cg.feeder_tb_voltage"),
    CargoConveyerSensor("cg.conveyer_tb_voltage"),
    CargoFeederBeamBroken("cg.feeder_tb_broken", true),
    CargoConveyerBeamBroken("cg.conveyer_tb_broken", true);

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
