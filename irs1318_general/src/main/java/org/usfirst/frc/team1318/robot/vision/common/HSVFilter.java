package org.usfirst.frc.team1318.robot.vision.common;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class HSVFilter
{
    private final Scalar lowerBound;
    private final Scalar upperBound;

    /**
     * Initializes a new instance of the HSVFilter class.
     * @param lowerBound of HSV to filter
     * @param upperBound of HSV to filter
     */
    public HSVFilter(Scalar lowerBound, Scalar upperBound)
    {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
    }

    /**
     * Filter the provided frame for HSVs within the provider bounds.
     * @param frame to filter
     * @return a matrix of 1s and 0s based on whether the pixel is within the provided HSV range or not, respectively.
     */
    public Mat filterHSV(Mat frame)
    {
        Mat sourceBGR = frame.clone();
        Imgproc.cvtColor(sourceBGR, frame, Imgproc.COLOR_BGR2HSV);
        sourceBGR.release();

        Mat sourceHSV = frame.clone();
        Core.inRange(sourceHSV, this.lowerBound, this.upperBound, frame);
        sourceHSV.release();

        return frame;
    }
}
