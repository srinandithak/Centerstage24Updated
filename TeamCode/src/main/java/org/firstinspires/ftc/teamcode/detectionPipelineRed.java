package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class detectionPipelineRed extends OpenCvPipeline {

    int Position;
    int maxValue;

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(290, 115);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(50, 128);
//    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253, 98);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 20;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//    Point region3_pointA = new Point(
//            REGION3_TOPLEFT_ANCHOR_POINT.x,
//            REGION3_TOPLEFT_ANCHOR_POINT.y);
//    Point region3_pointB = new Point(
//            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cr, region2_Cr, region3_Cr;
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    int avg1, avg2;
    //int avg3;

    // Volatile since accessed by OpMode thread w/o synchronization


    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);

        region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
//        region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));

    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        avg1 = (int) Core.mean(region1_Cr).val[0];
        avg2 = (int) Core.mean(region2_Cr).val[0];
//        avg3 = (int) Core.mean(region3_Cb).val[0];

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                region3_pointA, // First point which defines the rectangle
//                region3_pointB, // Second point which defines the rectangle
//                BLUE, // The color the rectangle is drawn in
//                2); // Thickness of the rectangle lines

        Imgproc.putText(input, ("" + avg1), region1_pointA, 1, 1, BLUE, 2);
        Imgproc.putText(input, ("" + avg2), region2_pointA, 1, 1, BLUE, 2);
//        Imgproc.putText(input, ("" + avg3), region3_pointA, 1, 1, BLUE, 3);

        /*
         * Find the max of the 3 averages
         */
        int max = Math.max(avg1, avg2);
//        int max = Math.max(max, avg3);
        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */

        //if its a small value, we assume its from region 0
        if (max < 141 ) {
            Position = 0;
        }
        else if (max == avg1) // Was it from region 1?
        {
            Position = 2;  // Record our analysis
            //draws rectangle on that region
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        } else if (max == avg2) //region 2?
        {
            Position = 1;
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1);

        }

        return input;


    }
    public int getAnalysis()
    {
        return Position;
    }

}
