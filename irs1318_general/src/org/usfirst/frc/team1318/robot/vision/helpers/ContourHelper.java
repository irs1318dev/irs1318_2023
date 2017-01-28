package org.usfirst.frc.team1318.robot.vision.helpers;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class ContourHelper
{
    /**
     * Find the largest contour in the frame
     * @param frame in which to look for contours
     * @return largest contour
     */
    public static MatOfPoint findLargestContour(Mat frame)
    {
        // find the contours using OpenCV API...
        Mat unused = new Mat();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(frame, contours, unused, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_KCOS);
        unused.release();

        // find the largest contour...
        double largestContourArea = 0.0;
        MatOfPoint largestContour = null;
        for (MatOfPoint contour : contours)
        {
            double area = Imgproc.contourArea(contour);
            if (area > largestContourArea)
            {
                if (largestContour != null)
                {
                    largestContour.release();
                }

                largestContour = contour;
                largestContourArea = area;
            }
            else
            {
                contour.release();
            }
        }

        return largestContour;
    }

    /**
     * Find the two largest contours in the frame
     * @param frame in which to look for contours
     * @return two largest contours, largest then second largest
     */
    public static MatOfPoint[] findTwoLargestContours(Mat frame)
    {
        // find the contours using OpenCV API...
        Mat unused = new Mat();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(frame, contours, unused, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_KCOS);
        unused.release();

        double largestContourArea = 0.0;
        MatOfPoint largestContour = null;

        double secondLargestContourArea = 0.0;
        MatOfPoint secondLargestContour = null;

        // find the two largest contours...
        for (MatOfPoint contour : contours)
        {
            double area = Imgproc.contourArea(contour);
            if (area > largestContourArea)
            {
                if (largestContour != null)
                {
                    if (secondLargestContour != null)
                    {
                        secondLargestContour.release();
                    }

                    secondLargestContour = largestContour;
                    secondLargestContourArea = largestContourArea;
                }

                largestContour = contour;
                largestContourArea = area;
            }
            else if (area > secondLargestContourArea)
            {
                if (secondLargestContour != null)
                {
                    secondLargestContour.release();
                }

                secondLargestContour = contour;
                secondLargestContourArea = area;
            }
            else
            {
                contour.release();
            }
        }

        return new MatOfPoint[] { largestContour, secondLargestContour };
    }

    /**
     * Find the contours in the frame, sorted from smallest to largest
     * @param frame in which to look for contours
     * @return sorted largest contour
     */
    public static MatOfPoint[] findSortedLargestContours(Mat frame)
    {
        // find the contours using OpenCV API...
        Mat unused = new Mat();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(frame, contours, unused, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_KCOS);
        unused.release();

        contours.sort(new Comparator<MatOfPoint>()
        {
            @Override
            public int compare(MatOfPoint o1, MatOfPoint o2)
            {
                double area1 = Imgproc.contourArea(o1);
                double area2 = Imgproc.contourArea(o2);
                if (area1 > area2)
                {
                    return 1;
                }
                else if (area1 < area2)
                {
                    return -1;
                }
                else
                {
                    return 0;
                }
            }
        });

        return (MatOfPoint[])contours.toArray();
    }

    /**
     * Find the center of mass for a contour using Moments.
     * http://docs.opencv.org/3.1.0/d8/d23/classcv_1_1Moments.html
     * @param contour to use
     * @return point representing the center of the contour
     */
    public static Point findCenterOfMass(MatOfPoint contour)
    {
        Moments moments = Imgproc.moments(contour);
        if (moments.m00 == 0.0)
        {
            return null;
        }

        return new Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
    }
}
