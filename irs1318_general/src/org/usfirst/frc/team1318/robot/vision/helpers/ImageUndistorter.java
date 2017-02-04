package org.usfirst.frc.team1318.robot.vision.helpers;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class ImageUndistorter
{
    private Mat mapX;
    private Mat mapY;

    /**
     * Initializes a new instance of the ImageUndistorter class.
     * For background, see http://docs.opencv.org/3.1.0/d4/d94/tutorial_camera_calibration.html
     */
    public ImageUndistorter()
    {
        Size size;
        Mat intrinsicMatrix;
        intrinsicMatrix = ImageUndistorter.build320x240Intrinsic();
        size = new Size(320, 240);

        Mat distCoeffs = ImageUndistorter.buildDistortion();

        // initialize mapX and mapY
        Mat mapX = new Mat(size, CvType.CV_32FC1);
        Mat mapY = new Mat(size, CvType.CV_32FC1);

        // unused:
        Mat R = new Mat();
        Mat newCameraMatrix = new Mat();

        Imgproc.initUndistortRectifyMap(intrinsicMatrix, distCoeffs, R, newCameraMatrix, size, CvType.CV_32FC1, mapX, mapY);

        this.mapX = mapX;
        this.mapY = mapY;

        intrinsicMatrix.release();
        distCoeffs.release();

        R.release();
        newCameraMatrix.release();
    }

    /**
     * Undistort the frame so that straight lines appear straight in the image
     * @param frame to undirsort
     * @return an non-distorted version of the provided frame
     */
    public Mat undistortFrame(Mat frame)
    {
        Mat source = frame.clone();

        Imgproc.remap(source, frame, this.mapX, this.mapY, Imgproc.INTER_LINEAR, Imgproc.WARP_FILL_OUTLIERS, new Scalar(0));
        source.release();

        return frame;
    }

    /**
     * Build an intrinsic matrix for the Axis M1011 camera at 320x240 resolution.
     * @return an intrinsic matrix
     */
    private static Mat build320x240Intrinsic()
    {
        Mat intrinsicMatrix = new Mat(3, 3, CvType.CV_32FC1);

        intrinsicMatrix.put(0, 0, 160); // focal length x
        intrinsicMatrix.put(0, 1, 0.0);
        intrinsicMatrix.put(0, 2, 160.0); // center x

        intrinsicMatrix.put(1, 0, 0.0);
        intrinsicMatrix.put(1, 1, 120); // focal length y [= x * h / w]
        intrinsicMatrix.put(1, 2, 120.0); // center y

        intrinsicMatrix.put(2, 0, 0.0);
        intrinsicMatrix.put(2, 1, 0.0);
        intrinsicMatrix.put(2, 2, 1.0); // flat z

        return intrinsicMatrix;
    }

    /**
     * Build an intrinsic matrix for the Axis M1011 camera at 640x480 resolution.
     * @return an intrinsic matrix
     */
    @SuppressWarnings("unused")
    private static Mat build640x480Intrinsic()
    {
        Mat intrinsicMatrix = new Mat(3, 3, CvType.CV_32FC1);

        intrinsicMatrix.put(0, 0, 320); // focal length x
        intrinsicMatrix.put(0, 1, 0.0);
        intrinsicMatrix.put(0, 2, 320.0); // center x

        intrinsicMatrix.put(1, 0, 0.0);
        intrinsicMatrix.put(1, 1, 240); // focal length y [= x * h / w]
        intrinsicMatrix.put(1, 2, 240.0); // center y

        intrinsicMatrix.put(2, 0, 0.0);
        intrinsicMatrix.put(2, 1, 0.0);
        intrinsicMatrix.put(2, 2, 1.0); // flat z

        return intrinsicMatrix;
    }

    /**
     * Build an distortion coefficient matrix for the Axis M1011 camera.
     * @return an distortion matrix
     */
    private static Mat buildDistortion()
    {
        Mat distortionCoeffs = new Mat(4, 1, CvType.CV_32FC1);

        double p_factor = 0.00;

        distortionCoeffs.put(0, 0, -0.055); // k1 * r^2
        distortionCoeffs.put(1, 0, 0.0); // k2 * r^4
        distortionCoeffs.put(2, 0, -p_factor); // tangential p1
        distortionCoeffs.put(3, 0, p_factor); // tangential p2

        return distortionCoeffs;
    }
}
