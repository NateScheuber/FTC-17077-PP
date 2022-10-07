package org.firstinspires.ftc.teamcode.Universal;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class RuthHardware {

    private static double intakeArmP = 1;
    private static double armSpeed = 0.5;
    private static int armPositionTolerance = 20;

    private static double liftP = 1;
    private static double liftSpeed = 0.75;
    private static double liftPositionTolerance = 20;

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

    private RevColorSensorV3 sensorRight = null;
    private RevColorSensorV3 sensorLeft = null;


    ElapsedTime scoreTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public RuthHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
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

        RevColorSensorV3 sensorRight = myOpMode.hardwareMap.get(RevColorSensorV3.class, "sensorRight");
        RevColorSensorV3 sensorLeft = myOpMode.hardwareMap.get(RevColorSensorV3.class, "sensorleft");

        sensorRight.initialize();
        sensorLeft.initialize();

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        intakeArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeArm.setPositionPIDFCoefficients(intakeArmP);
        intakeArm.setTargetPositionTolerance(armPositionTolerance);

        liftMaster.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSlave.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMaster.setPositionPIDFCoefficients(liftP);
        liftSlave.setPositionPIDFCoefficients(liftP);

        scoreTimer.reset();
    }

    public void driveFieldOriented(double X, double Y, double R, double heading){

    }

    public void driveRobotOriented(double X, double Y, double R){
        frontRight.setPower(Y-X-R);
        backRight.setPower(Y+X-R);
        frontLeft.setPower(Y-X+R);
        backLeft.setPower(Y+X+R);
    }

    public void driveSlowMo(boolean forward, boolean backward, boolean right, boolean left){
        if(forward){
            driveRobotOriented(0.25, 0, 0);
        }
        else if(backward){
            driveRobotOriented(-0.25,0,0);
        }

        if(right){
            driveRobotOriented(0, 0.25, 0);
        }
        else if(left){
            driveRobotOriented(0, -0.25, 0);
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
                intakeArm.setTargetPosition(-100);
                break;

            case intakeDown:
                intakeArm.setTargetPosition(-110);

            case score:
                intakeArm.setTargetPosition(50);
                break;
        }
        intakeArm.setPower(armSpeed);

        telemetry.addData("Arm Position", intakeArm.getCurrentPosition());
        telemetry.update();
    }

    public boolean intakeSensor(){
        return (sensorRight.getDistance(DistanceUnit.MM) < 10 & sensorLeft.getDistance(DistanceUnit.MM) < 10);
    }

    public void intake(double speed){
        intakeLeft.setPower(speed);
        intakeRight.setPower(speed);
    }

    public void pickUp(boolean rightBumper, boolean leftBumper){
        intakeTimer.reset();
        if(rightBumper && intakeSensor()){
            arm("intakeDown");
            intake(1);
            if(intakeTimer.time()>250){
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

        currentHeight = liftPosition.valueOf(height);

            switch(currentHeight){
                case home:
                    liftMaster.setTargetPosition(0);
                    liftSlave.setTargetPosition(liftMaster.getTargetPosition());
                    break;

                case low:
                    liftMaster.setTargetPosition(100);
                    liftSlave.setTargetPosition(liftMaster.getTargetPosition());
                    break;

                case medium:
                    liftMaster.setTargetPosition(200);
                    liftSlave.setTargetPosition(liftMaster.getTargetPosition());
                    break;

                case high:
                    liftMaster.setTargetPosition(300);
                    liftSlave.setTargetPosition(liftMaster.getTargetPosition());
                    break;
            }


        liftMaster.setPower(liftSpeed);
        liftSlave.setPower(liftSpeed);
        telemetry.addData("lift Position", liftMaster.getCurrentPosition());
        telemetry.update();
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
            arm("front");
        }
        else if(scoreTimer.time()<600){
            lift("intakeUp");
        }
    }
}
