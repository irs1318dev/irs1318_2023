package org.usfirst.frc.team1318.robot.vision.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team1318.robot.vision.VisionConstants;
import org.usfirst.frc.team1318.robot.vision.helpers.ContourHelper;
import org.usfirst.frc.team1318.robot.vision.helpers.HSVFilter;
import org.usfirst.frc.team1318.robot.vision.helpers.ImageUndistorter;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.vision.VisionPipeline;

public class HSVGearCenterPipeline implements VisionPipeline
{
    private final boolean shouldUndistort;
    private final ImageUndistorter undistorter;
    private final HSVFilter hsvFilter;
    private final Timer timer;

    private final CvSource frameInput;
    private final CvSource hsvOutput;
    private final CvSource finalOutput;

    // measured values
    private Point largestCenter;
    private Point secondLargestCenter;

    private Double thetaXOffsetMeasured;

    private Double thetaXOffsetDesired;
    private Double distanceFromCam;
    private Double distanceFromRobot;

    // FPS Measurement
    private long analyzedFrameCount;
    private double lastMeasuredTime;
    private double lastFpsMeasurement;

    // active status
    private boolean isActive;

    /**
     * Initializes a new instance of the HSVGearCenterPipeline class.
     * @param shouldUndistort whether to undistort the image or not
     */
    public HSVGearCenterPipeline(boolean shouldUndistort)
    {
        this.shouldUndistort = shouldUndistort;

        this.undistorter = new ImageUndistorter();
        this.hsvFilter = new HSVFilter(VisionConstants.LIFECAM_HSV_FILTER_LOW, VisionConstants.LIFECAM_HSV_FILTER_HIGH);

        this.largestCenter = null;
        this.secondLargestCenter = null;

        this.thetaXOffsetMeasured = null;

        this.thetaXOffsetDesired = null;
        this.distanceFromCam = null;
        this.distanceFromRobot = null;

        this.analyzedFrameCount = 0;
        this.timer = new Timer();
        this.timer.start();
        this.lastMeasuredTime = this.timer.get();

        this.isActive = true;

        if (VisionConstants.DEBUG && VisionConstants.DEBUG_OUTPUT_FRAMES)
        {
            this.frameInput = CameraServer.getInstance().putVideo("input", VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);
            this.hsvOutput = CameraServer.getInstance().putVideo("hsv", VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);
            this.finalOutput = CameraServer.getInstance().putVideo("final", VisionConstants.LIFECAM_CAMERA_RESOLUTION_X, VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y);
        }
        else
        {
            this.frameInput = null;
            this.hsvOutput = null;
            this.finalOutput = null;
        }
    }

    /**
     * Process a single image frame
     * @param frame image to analyze
     */
    @Override
    public void process(Mat image)
    {
        if (!this.isActive)
        {
            return;
        }

        this.analyzedFrameCount++;
        if (VisionConstants.DEBUG && VisionConstants.DEBUG_PRINT_OUTPUT && this.analyzedFrameCount % VisionConstants.DEBUG_FPS_AVERAGING_INTERVAL == 0)
        {
            double now = this.timer.get();
            double elapsedTime = now - this.lastMeasuredTime;

            this.lastFpsMeasurement = ((double)VisionConstants.DEBUG_FPS_AVERAGING_INTERVAL) / elapsedTime;
            this.timer.reset();
            this.lastMeasuredTime = this.timer.get();
        }

        // first, undistort the image.
        Mat undistortedImage;
        if (this.shouldUndistort)
        {
            image = this.undistorter.undistortFrame(image);
        }

        if (VisionConstants.DEBUG)
        {
            if (VisionConstants.DEBUG_SAVE_FRAMES && this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0)
            {
                Imgcodecs.imwrite(String.format("%simage%d-1.undistorted.jpg", VisionConstants.DEBUG_OUTPUT_FOLDER, this.analyzedFrameCount), image);
            }

            if (VisionConstants.DEBUG_OUTPUT_FRAMES)
            {
                this.frameInput.putFrame(image);
            }
        }

        // save the undistorted image for possible output later...
        if (this.shouldUndistort)
        {
            undistortedImage = image.clone();
        }
        else
        {
            undistortedImage = image;
        }

        // second, filter HSV
        image = this.hsvFilter.filterHSV(image);
        if (VisionConstants.DEBUG)
        {
            if (VisionConstants.DEBUG_SAVE_FRAMES && this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0)
            {
                Imgcodecs.imwrite(String.format("%simage%d-2.hsvfiltered.jpg", VisionConstants.DEBUG_OUTPUT_FOLDER, this.analyzedFrameCount), image);
            }

            if (VisionConstants.DEBUG_OUTPUT_FRAMES)
            {
                this.hsvOutput.putFrame(image);
            }
        }

        // third, find the largest contour.
        MatOfPoint[] largestContours = ContourHelper.findTwoLargestContours(image, VisionConstants.CONTOUR_MIN_AREA);
        MatOfPoint largestContour = largestContours[0];
        MatOfPoint secondLargestContour = largestContours[1];

        if (largestContour == null)
        {
            if (VisionConstants.DEBUG && VisionConstants.DEBUG_PRINT_OUTPUT && VisionConstants.DEBUG_PRINT_ANALYZER_DATA)
            {
                System.out.println("could not find any contour");
            }
        }

        // fourth, find the center of mass for the largest two contours
        Point largestCenterOfMass = null;
        Point secondLargestCenterOfMass = null;
        Rect largestBoundingRect = null;
        Rect secondLargestBoundingRect = null;
        if (largestContour != null)
        {
            largestCenterOfMass = ContourHelper.findCenterOfMass(largestContour);
            largestBoundingRect = Imgproc.boundingRect(largestContour);
            largestContour.release();
        }

        if (secondLargestContour != null)
        {
            secondLargestCenterOfMass = ContourHelper.findCenterOfMass(secondLargestContour);
            secondLargestBoundingRect = Imgproc.boundingRect(secondLargestContour);
            secondLargestContour.release();
        }

        if (VisionConstants.DEBUG)
        {
            if (VisionConstants.DEBUG_PRINT_OUTPUT && VisionConstants.DEBUG_PRINT_ANALYZER_DATA)
            {
                if (largestCenterOfMass == null)
                {
                    System.out.println("couldn't find the center of mass!");
                }
                else
                {
                    System.out.println(String.format("Center of mass: %f, %f", largestCenterOfMass.x, largestCenterOfMass.y));
                }

                if (secondLargestCenterOfMass == null)
                {
                    System.out.println("couldn't find the center of mass!");
                }
                else
                {
                    System.out.println(String.format("Center of mass: %f, %f", secondLargestCenterOfMass.x, secondLargestCenterOfMass.y));
                }
            }

            if (largestCenterOfMass != null &&
                ((this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0 && VisionConstants.DEBUG_SAVE_FRAMES) || VisionConstants.DEBUG_OUTPUT_FRAMES))
            {
                Imgproc.circle(undistortedImage, largestCenterOfMass, 2, new Scalar(0, 0, 255), -1);
                if (secondLargestCenterOfMass != null)
                {
                    Imgproc.circle(undistortedImage, secondLargestCenterOfMass, 2, new Scalar(0, 0, 128), -1);
                }

                if (this.analyzedFrameCount % VisionConstants.DEBUG_FRAME_OUTPUT_GAP == 0 && VisionConstants.DEBUG_SAVE_FRAMES)
                {
                    Imgcodecs.imwrite(String.format("%simage%d-3.redrawn.jpg", VisionConstants.DEBUG_OUTPUT_FOLDER, this.analyzedFrameCount), undistortedImage);
                }

                if (VisionConstants.DEBUG_OUTPUT_FRAMES)
                {
                    this.finalOutput.putFrame(image);
                }
            }
        }

        // finally, record the centers of mass
        this.largestCenter = largestCenterOfMass;
        this.secondLargestCenter = secondLargestCenterOfMass;

        undistortedImage.release();

        // GEAR CALCULATIONS
        if (this.largestCenter == null && this.secondLargestCenter == null)
        {
            this.thetaXOffsetDesired = null;
            this.distanceFromCam = null;
            this.distanceFromRobot = null;

            this.thetaXOffsetMeasured = null;

            return;
        }

        Point gearMarkerCenter;
        Rect boundingRect;
        if (this.largestCenter != null && this.secondLargestCenter != null && this.largestCenter.x < this.secondLargestCenter.x)
        {
            gearMarkerCenter = this.secondLargestCenter;
            boundingRect = secondLargestBoundingRect;
        }
        else
        {
            gearMarkerCenter = this.largestCenter;
            boundingRect = largestBoundingRect;
        }

        int gearMarkerHeight = boundingRect.height;
        if (gearMarkerHeight == 0)
        {
            return;
        }

        // Find desired data
        double xOffsetMeasured = gearMarkerCenter.x - VisionConstants.LIFECAM_CAMERA_CENTER_WIDTH;
        this.thetaXOffsetMeasured = xOffsetMeasured * VisionConstants.LIFECAM_CAMERA_FIELD_OF_VIEW_X / (double)VisionConstants.LIFECAM_CAMERA_RESOLUTION_X;

        this.distanceFromCam = ((VisionConstants.REAL_GEAR_RETROREFLECTIVE_TAPE_HEIGHT) / (Math.tan(VisionConstants.LIFECAM_CAMERA_FIELD_OF_VIEW_Y_RADIANS)))
            * ((double)VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y / (double)gearMarkerHeight);
        this.distanceFromRobot = this.distanceFromCam * Math.cos(this.thetaXOffsetMeasured * VisionConstants.ANGLE_TO_RADIANS);
        this.thetaXOffsetDesired = Math.asin(VisionConstants.GEAR_CAMERA_OFFSET_FROM_CENTER / this.distanceFromCam) * VisionConstants.RADIANS_TO_ANGLE;
    }

    public void setActivation(boolean isActive)
    {
        this.isActive = isActive;
    }

    public Point getCenter()
    {
        return this.largestCenter;
    }

    public Double getThetaXOffsetDesired()
    {
        return this.thetaXOffsetDesired;
    }

    public Double getThetaXOffsetMeasured()
    {
        return this.thetaXOffsetMeasured;
    }

    public Double getDistanceFromCam()
    {
        return this.distanceFromCam;
    }

    public Double getDistanceFromRobot()
    {
        return this.distanceFromRobot;
    }

    public double getFps()
    {
        return this.lastFpsMeasurement;
    }
}
