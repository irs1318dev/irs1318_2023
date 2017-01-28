package org.usfirst.frc.team1318.robot.vision;

import org.opencv.core.Scalar;

public class VisionConstants
{
    // Debug output settings:
    public static final boolean DEBUG = true;
    public static final boolean DEBUG_PRINT_OUTPUT = true;
    public static final boolean DEBUG_PRINT_ANALYZER_DATA = false;
    public static final int DEBUG_FPS_AVERAGING_INTERVAL = 30;
    public static final boolean DEBUG_FRAME_OUTPUT = false;
    public static final int DEBUG_FRAME_OUTPUT_GAP = 30; // the number of frames to wait between saving debug image output
    public static final String DEBUG_OUTPUT_FOLDER = "/C/vision/";

    // Settings for AXIS IP-based Camera
    public static final String CAMERA_IP_ADDRESS = "10.13.18.11";
    public static final String CAMERA_USERNAME_PASSWORD = "root:1318";
    public static final int CAMERA_RESOLUTION_X = 320;
    public static final int CAMERA_RESOLUTION_Y = 240;
    public static final double CAMERA_ANGLE_OF_VIEW = 50.0; // note that documentation says 47 degrees, so we'll have to see whether this is accurate enough.
    public static final int CAMERA_CENTER_WIDTH = VisionConstants.CAMERA_RESOLUTION_X / 2; // distance from center to left/right sides in pixels
    public static final int CAMERA_CENTER_HEIGHT = VisionConstants.CAMERA_RESOLUTION_Y / 2; // distance from center to top/bottom in pixels
    public static final double CAMERA_CENTER_VIEW_ANGLE = VisionConstants.CAMERA_ANGLE_OF_VIEW / 2.0;

    // HSV Filtering constants
    public static final Scalar HSV_FILTER_LOW = new Scalar(85, 65, 65);
    public static final Scalar HSV_FILTER_HIGH = new Scalar(90, 255, 255);
}
