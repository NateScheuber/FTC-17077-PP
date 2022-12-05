package org.firstinspires.ftc.teamcode.Universal;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
public class Ruth_v2_Hardware {
    private LinearOpMode myOpMode = null;



    public static double intakeArmP = 3.5;
    public static double armSpeed = 0.25;
    public static int armPositionTolerance = 10;

    boolean clawToggle = true;
    boolean clawPosition = false;
    boolean isclawFlipped = false;

    public static double clawOpen = 0.2;
    public static double clawClosed = 0.0;
    public static double clawRotateUR = 0.1;
    public static double clawRotateUD = 0.75;
    public static double clawLevel = 0.225;
    public static double clawFlipped = 0.0;
    public static double clawLowLevel = 0.36;

    public static double liftP = 4;
    public static double liftSpeed = 0.8;
    public static int liftPositionTolerance = 5;

    public static int level1 = -0;
    public static int level2 = -360;
    public static int level3 = -1100;
    public static int liftLevel = 0;

    public enum liftPosition{
        home,
        low,
        medium,
        high
    }
    liftPosition currentHeight = liftPosition.home;

    public enum armPosition{
        home,
        low,
        up,
        back,
    }
    armPosition currentArmPosition = armPosition.home;

    private DcMotorEx frontRight  = null;
    private DcMotorEx frontLeft   = null;
    private DcMotorEx backRight   = null;
    private DcMotorEx backLeft    = null;
    private DcMotorEx liftMaster  = null;
    private DcMotorEx liftSlave   = null;
    private DcMotorEx intakeArm   = null;

    private ServoImplEx claw = null;
    private Servo clawRotate = null;
    private Servo clawFlip = null;
    private Servo wheelieA = null;
    private Servo wheelieB = null;

    ElapsedTime scoreTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public Ruth_v2_Hardware(LinearOpMode opmode) {
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

        claw = myOpMode.hardwareMap.get(ServoImplEx.class, "claw");
        clawRotate = myOpMode.hardwareMap.get(Servo.class, "clawRotate");
        clawFlip = myOpMode.hardwareMap.get(Servo.class, "clawFlip");

        wheelieA = myOpMode.hardwareMap.get(Servo.class, "wheelieA");
        wheelieB = myOpMode.hardwareMap.get(Servo.class, "wheelieB");

        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setTargetPosition(0);
        intakeArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //intakeArm.setPositionPIDFCoefficients(intakeArmP);
        intakeArm.setTargetPositionTolerance(armPositionTolerance);
        intakeArm.setPower(1);

        liftMaster.setTargetPosition(0);
        liftSlave.setTargetPosition(0);
        liftMaster.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSlave.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //liftMaster.setPositionPIDFCoefficients(liftP);
        //liftSlave.setPositionPIDFCoefficients(liftP);
        liftMaster.setTargetPositionTolerance(liftPositionTolerance);
        liftMaster.setTargetPositionTolerance(liftPositionTolerance);

        claw.setPosition(0.4);
        clawRotate.setPosition(clawRotateUR);
        clawFlip.setPosition(clawLevel);

        scoreTimer.reset();
        intakeTimer.reset();
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


    public void arm(String position, int offset){

        currentArmPosition = Ruth_v2_Hardware.armPosition.valueOf(position);

        switch(currentArmPosition){
            case home:
                intakeArm.setTargetPosition(0-offset);
                break;

            case back:
                intakeArm.setTargetPosition(600-offset);
                break;

            case up:
                intakeArm.setTargetPosition(325-offset);
                break;

            case low:
                intakeArm.setTargetPosition(-350-offset);
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
    public boolean armBusy(){return intakeArm.isBusy();}

    //claw open/closed
    public void claw(double position){
        claw.setPosition(position);
    }
    public void clawFlip(double position){
        clawFlip.setPosition(position);
    }


    //lift control
    public void lift(String height) {

        liftMaster.setPositionPIDFCoefficients(liftP);
        liftSlave.setPositionPIDFCoefficients(liftP);
        currentHeight = liftPosition.valueOf(height);

        switch(currentHeight){
            case home:
                liftMaster.setTargetPosition(0);
                liftSlave.setTargetPosition(0);
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
    public boolean liftBusy(){return liftMaster.isBusy();}

    public void clawProtection(){
        if(liftBusy() || armBusy()){
            claw.setPosition(clawClosed);
        }
    }

    public void clawRotate(){
        if(liftCurrentPosition()<-200){
            clawRotate.setPosition(clawRotateUD);
        }
        else if(liftCurrentPosition()>-200){
            clawRotate.setPosition(clawRotateUR);
        }
    }

}
