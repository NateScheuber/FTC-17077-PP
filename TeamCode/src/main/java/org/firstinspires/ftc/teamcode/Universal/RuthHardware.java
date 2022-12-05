package org.firstinspires.ftc.teamcode.Universal;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.VIsion.Signal_Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
@Config
public class RuthHardware {



    private static double intakeArmP = 2.5;
    private static double armSpeed = 0.5;
    private static int armPositionTolerance = 10;

    public static double liftP = 1;
    public static double liftSpeed = 0.5;
    public static int liftPositionTolerance = 20;

    public static int level1 = 700;
    public static int level2 = 1400;
    public static int level3 = 2140;
    public static int liftLevel = 0;

    public enum liftPosition{
        home,
        low,
        medium,
        high
    }
    liftPosition currentHeight = liftPosition.home;

    public enum armPosition{
        safe,
        intakeUp,
        intakeDown,
        back,
        score
    }
    armPosition currentArmPosition = armPosition.safe;


    private LinearOpMode myOpMode = null;

    private DcMotorEx frontRight  = null;
    private DcMotorEx frontLeft   = null;
    private DcMotorEx backRight   = null;
    private DcMotorEx backLeft    = null;
    private DcMotorEx liftMaster  = null;
    private DcMotorEx liftSlave   = null;
    private DcMotorEx intakeArm   = null;

    private CRServo intakeLeft = null;
    private CRServo intakeRight = null;


    ElapsedTime scoreTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public RuthHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontRight = myOpMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = myOpMode.hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "backLeft");

        liftMaster = myOpMode.hardwareMap.get(DcMotorEx.class, "liftMaster");
        liftSlave = myOpMode.hardwareMap.get(DcMotorEx.class, "liftSlave");
        intakeArm = myOpMode.hardwareMap.get(DcMotorEx.class, "intakeArm");

        intakeLeft = myOpMode.hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = myOpMode.hardwareMap.get(CRServo.class, "intakeRight");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        intakeArm.setTargetPosition(0);
        intakeArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeArm.setPositionPIDFCoefficients(intakeArmP);
        intakeArm.setTargetPositionTolerance(armPositionTolerance);

        liftMaster.setTargetPosition(0);
        liftSlave.setTargetPosition(0);
        liftMaster.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSlave.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMaster.setPositionPIDFCoefficients(liftP);
        liftSlave.setPositionPIDFCoefficients(liftP);
        liftMaster.setTargetPositionTolerance(liftPositionTolerance);
        liftMaster.setTargetPositionTolerance(liftPositionTolerance);

        scoreTimer.reset();
    }

    public void driveRobotOriented(double X, double Y, double R){
        frontRight.setPower(Y-X-R);
        backRight.setPower(Y+X-R);
        frontLeft.setPower(Y+X+R);
        backLeft.setPower(Y-X+R);
    }

    public void driveFieldOriented(double X, double Y, double R, double heading){
        double h = Math.sqrt(X*X+Y*Y);
        double angle = Math.atan2(X,Y)-(heading/522);
        double xPower = (h*Math.sin(angle));
        double yPower = (h*Math.cos(angle));
        driveRobotOriented(xPower, yPower, R);
    }

    public void driveSlowMo(boolean forward, boolean backward, boolean right, boolean left){
        if(forward){
            driveRobotOriented(0, 0.25, 0);
        }
        else if(backward){
            driveRobotOriented(0,-0.25,0);
        }

        if(right){
            driveRobotOriented(0.25, 0, 0);
        }
        else if(left){
            driveRobotOriented(-0.25, 0, 0);
        }
    }

    public void arm(String position){

        currentArmPosition = armPosition.valueOf(position);

        switch(currentArmPosition){
            case safe:
                intakeArm.setTargetPosition(0);
                break;

            case back:
                intakeArm.setTargetPosition(100);
                break;

            case intakeUp:
                intakeArm.setTargetPosition(-180);
                break;

            case intakeDown:
                intakeArm.setTargetPosition(-300);
                break;

            case score:
                intakeArm.setTargetPosition(200);
                break;
        }
        intakeArm.setPower(armSpeed);
    }

    public int armCurrentPosition(){
        return intakeArm.getCurrentPosition();
    }

    public int armTargetPosition(){
        return intakeArm.getTargetPosition();
    }

    public boolean intakeSensor(double right, double left){
        return right < 30 && left < 30;
    }

    public void intake(double speed){
        intakeLeft.setPower(-speed);
        intakeRight.setPower(speed);
    }

    public void pickUp(boolean rightBumper, boolean leftBumper, double distanceRight, double distanceLeft){
        intakeTimer.reset();
        if(rightBumper && intakeSensor(distanceRight, distanceLeft)){
            arm("intakeDown");
            intake(1);
            if(intakeTimer.time()>1000){
                arm("safe");
                intake(0);
            }
        }
        else if(leftBumper){
            arm("intakeDown");
            if(intakeTimer.time()>250){
                intake(-1);
            }
            else if(intakeTimer.time()>500){
                arm("safe");
                intake(0);
            }
        }
    }


    public void lift(String height) {

        liftMaster.setPositionPIDFCoefficients(liftP);
        liftSlave.setPositionPIDFCoefficients(liftP);
        currentHeight = liftPosition.valueOf(height);

            switch(currentHeight){
                case home:
                    liftMaster.setTargetPosition(10);
                    liftSlave.setTargetPosition(10);
                    liftLevel = 0;
                    break;

                case low:
                    liftMaster.setTargetPosition(level1);
                    liftSlave.setTargetPosition(level1);
                    liftLevel = 1;
                    break;

                case medium:
                    liftMaster.setTargetPosition(level2);
                    liftSlave.setTargetPosition(level2);
                    liftLevel = 2;
                    break;

                case high:
                    liftMaster.setTargetPosition(level3);
                    liftSlave.setTargetPosition(level3);
                    liftLevel = 3;
                    break;
            }

        liftMaster.setPower(liftSpeed);
        liftSlave.setPower(liftSpeed);

        myOpMode.telemetry.addData("lift Position", liftMaster.getCurrentPosition());
        myOpMode.telemetry.update();
    }

    public int liftTargetPosition(){
        return liftMaster.getTargetPosition();
    }

    public int liftCurrentPosition(){
        return liftMaster.getCurrentPosition();
    }



    public String liftLevel(){
        return currentHeight.toString();
    }

    public void score(){
        scoreTimer.reset();
        if(scoreTimer.time()<300){
            intakeLeft.setPower(1);
            intakeRight.setPower(-1);
        }
        else if(scoreTimer.time()<500){
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            arm("safe");
        }
        else if(scoreTimer.time()<600){
            lift("home");
        }
    }

    public void initCamera(){
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        WebcamName webcamName = myOpMode.hardwareMap.get(WebcamName.class, "webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(new Signal_Pipeline());
    }
}