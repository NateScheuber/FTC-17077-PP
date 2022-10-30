package org.firstinspires.ftc.teamcode.Auto.VIsion;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Signal_Pipeline extends OpenCvPipeline {

    Telemetry telemetry;

    /*
    public Signal_Pipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

     */

    public static int getAnalysis;
    public int rotation = 1;


    static final Point topLeft = new Point (320,140);
    static final Point bottomRight = new Point (340, 160);

    Mat region;
    Mat HSV = new Mat();
    Mat H = new Mat();
    int avg;


    static final Scalar WHITE = new Scalar(255, 255, 255);
    static final Scalar YELLOW = new Scalar(255, 255, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BlACK = new Scalar(0, 0, 0);



    void inputToH(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, H, 1);
    }



    public void init(Mat firstFrame){
        inputToH(firstFrame);
        region = H.submat(new Rect(topLeft, bottomRight));
    }

    @Override
    public Mat processFrame(Mat input) {
            inputToH(input);

        telemetry.addData("Hue", avg);
        telemetry.update();


        avg = (int) Core.mean(region).val[0];



            Imgproc.rectangle(
                    input,
                    topLeft,
                    bottomRight,
                    WHITE,
                    2
            );
         if(avg >= 0 && avg < 90){
             Imgproc.rectangle(
                     input,
                     topLeft,
                     bottomRight,
                     YELLOW,
                     2
             );
             rotation = 1;
         }
         else if(avg >= 90 && avg < 180){
             Imgproc.rectangle(
                     input,
                     topLeft,
                     bottomRight,
                     GREEN,
                     2
             );
             rotation = 2;
         }
         else if(avg >= 180 && avg <= 255){
             Imgproc.rectangle(
                     input,
                     topLeft,
                     bottomRight,
                     BlACK,
                     2
             );
             rotation = 3;
         }
        return input;

    }
    public int getAnalysis(){
        return rotation;
    }

}
