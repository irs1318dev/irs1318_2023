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
    OffboardVisionRRTargetDistance("vision.rr_distance"),
    OffboardVisionRRTargetHorizontalAngle("vision.rr_horizontalAngle"),
    OffboardVisionAprilTagXOffset("vision.atXOffset"),
    OffboardVisionAprilTagYOffset("vision.atYOffset"),
    OffboardVisionAprilTagZOffset("vision.atZOffset"),
    OffboardVisionAprilTagYaw("vision.atYaw"),
    OffboardVisionAprilTagPitch("vision.atPitch"),
    OffboardVisionAprilTagRoll("vision.atRoll"),
    OffboardVisionAprilTagId("vision.atId"),
    OffboardVisionProcessingMode("vision.processingMode"),
    OffboardVisionEnableStream("vision.enableStream"),
    OffboardVisionMissedHeartbeats("vision.missedHeartbeats"),
    PowerCurrent("power.curr"),
    PowerCurrentFloatingAverage("power.currFltAvg"),
    PowerBatteryVoltage("power.battV"),
    NavxStartingAngle("navx.startingAngle"),
    PigeonState("pigeon.state"),
    PigeonYaw("pigeon.yaw", true),
    PigeonPitch("pigeon.pitch", true),
    PigeonPitchOffset("pigeon.pitchOffset"),
    PigeonRollOffset("pigeon.rollOffset"),
    PigeonYawOffset("pigeon.yawOffset"),
    PigeonRoll("pigeon.roll", true),
    PigeonStartingYaw("pigeon.startingYaw"),
    PigeonYawRate("pigeon.yawRate", true),
    PigeonPitchRate("pigeon.pitchRate", true),
    PigeonRollRate("pigeon.rollRate", true),
    NavxConnected("navx.isConnected"),
    NavxAngle("navx.angle"),
    NavxPitch("navx.pitch"),
    NavxRoll("navx.roll"),
    NavxYaw("navx.yaw"),
    NavxX("navx.x"),
    NavxY("navx.y"),
    NavxZ("navx.z"),

    DriveTrainDesiredAngle("dt.angle_goal"),
    DriveTrainAngle("dt.angle"),
    DriveTrainXPosition("dt.xpos", true),
    DriveTrainYPosition("dt.ypos", true),
    DriveTrainXPositionGoal("dt.xpos_goal"),
    DriveTrainYPositionGoal("dt.ypos_goal"),
    DriveTrainAngleGoal("dt.angle_pathgoal"),
    DriveTrainXVelocityGoal("dt.xvel_goal"),
    DriveTrainYVelocityGoal("dt.yvel_goal"),
    DriveTrainAngleVelocityGoal("dt.anglevel_goal"),
    DriveTrainFieldOriented("dt.field_oriented"),
    DriveTrainMaintainOrientation("dt.maintain_orientation"),

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

    ArmFKXPosition("a.fk_x_pos"),
    ArmFKZPosition("a.fk_z_pos"),
    ArmDesiredXPosition("a.ik_x_desired"),
    ArmDesiredZPosition("a.ik_z_desired"),
    ikXAdjustment("a.ik_x_adj"),
    ikZAdjustment("a.ik_z_adj"),
    ArmLowerStalled("a.low_stalled"),
    ArmLowerLeftDesiredPosition("a.low_left_pos_desired"),
    ArmLowerLeftPosition("a.low_left_pos"),
    ArmLowerLeftVelocity("a.low_left_vel"),
    ArmLowerLeftVelocityAverage("a.low_left_vel_avg"),
    ArmLowerLeftError("a.low_left_err"),
    ArmLowerLeftPower("a.low_left_power"),
    ArmLowerRightDesiredPosition("a.low_right_pos_desired"),
    ArmLowerRightPosition("a.low_right_pos"),
    ArmLowerRightVelocity("a.low_right_vel"),
    ArmLowerRightVelocityAverage("a.low_right_vel_avg"),
    ArmLowerRightError("a.low_right_err"),
    ArmLowerRightPower("a.low_right_power"),
    ArmUpperStalled("a.up_stalled"),
    ArmUpperDesiredPosition("a.up_pos_desired"),
    ArmUpperPosition("a.up_pos"),
    ArmUpperVelocity("a.up_vel"),
    ArmUpperVelocityAverage("a.up_vel_avg"),
    ArmUpperError("a.up_err"),
    ArmUpperPower("a.up_power"),
    ArmIntakePower("a.intake_power"),
    ArmIntakeExtended("a.intake_extended"),

    CompressorPreassure("com.pres");

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
