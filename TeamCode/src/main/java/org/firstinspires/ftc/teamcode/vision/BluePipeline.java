package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BluePipeline extends OpenCvPipeline {

    Mat mat;
    Telemetry telemetry;
    public BluePipeline(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(input,mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(110,50,50);
        Scalar highHSV = new Scalar(130,255,255);

        Core.inRange(mat,lowHSV,highHSV,mat);

        List<MatOfPoint> cnList = new ArrayList<>();
        Imgproc.findContours(mat, cnList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect closestBlue = new Rect(new Point(0,0), new Point(1,1));
        for (MatOfPoint cn : cnList)
        {

            Rect rect = Imgproc.boundingRect(cn);
            if (rect.area() > closestBlue.area()) {
                closestBlue = rect;
            }

        }
        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat, closestBlue, new Scalar(255,255,255));
        telemetry.addData("Cone Area: ", closestBlue.area());

        return input;
    }


}