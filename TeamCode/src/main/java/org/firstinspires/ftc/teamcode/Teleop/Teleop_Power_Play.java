package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Universal.RuthHardware;

import java.util.List;
import java.util.Objects;

@Config
@TeleOp (name = "Teleop")
public class Teleop_Power_Play extends LinearOpMode {

    int liftPosition = 0;

    private final RevColorSensorV3 sensorRight = null;
    private final RevColorSensorV3 sensorLeft = null;


    Orientation angles;

    private BNO055IMU imu = null;

    RuthHardware Ruth = new RuthHardware(this);




    @Override
    public void runOpMode() throws InterruptedException {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Ruth.init();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        RevColorSensorV3 sensorRight = hardwareMap.get(RevColorSensorV3.class, "sensorRight");
        RevColorSensorV3 sensorLeft = hardwareMap.get(RevColorSensorV3.class, "sensorLeft");
        imu.initialize(parameters);


        Ruth.arm("safe");
        waitForStart();
        while(opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double driveX = gamepad1.right_stick_x;
            double driveY = -gamepad1.right_stick_y;
            double driveR = gamepad1.left_stick_x + (gamepad1.right_trigger * 0.25) - (gamepad1.left_trigger * 0.25);
            double heading = angles.firstAngle;


            //dt control
            Ruth.driveRobotOriented(driveX, driveY, driveR);
            Ruth.driveSlowMo(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left);

            //intake control

            if(gamepad1.right_bumper){
                Ruth.intake(1);
                Ruth.arm("intakeDown");
            }
            else if(gamepad1.left_bumper){
                Ruth.arm("intakeDown");
                if(Ruth.intakeSensor(sensorRight.getDistance(DistanceUnit.MM), sensorLeft.getDistance(DistanceUnit.MM))){
                    Ruth.intake(-1);
                    Ruth.arm("intakeUp");
                }
            }
            else{
                Ruth.arm("safe");
                Ruth.intake(0.04);
            }


            /*
            if(gamepad1.right_bumper){
                Ruth.intake(1);
                if(Ruth.intakeSensor(sensorRight.getDistance(DistanceUnit.MM), sensorLeft.getDistance(DistanceUnit.MM))){
                    Ruth.arm("intakeDown");
                }
                else{
                    Ruth.arm("intakeUp");
                }
            }
            else if(gamepad1.left_bumper){
                Ruth.arm("intakeUp");
                if(Ruth.intakeSensor(sensorRight.getDistance(DistanceUnit.MM), sensorLeft.getDistance(DistanceUnit.MM))){
                    Ruth.intake(-1);
                }
            }
            else if(Objects.equals(Ruth.liftLevel(), "home")){
                Ruth.arm("intakeUp");
                Ruth.intake(0.04);
            }

            else{
                Ruth.arm("safe");
                Ruth.intake(0.04);
            }

             */


            //lift control
            if(gamepad2.dpad_down){
                Ruth.lift("home");

                liftPosition = 0;
            }
            else if(gamepad2.dpad_left){
                Ruth.lift("low");
                liftPosition = 1;
            }
            else if(gamepad2.dpad_right){
                Ruth.lift("medium");
                liftPosition = 2;
            }
            else if(gamepad2.dpad_up){
                Ruth.lift("high");
                liftPosition = 3;
            }

            //score
            if(Ruth.liftCurrentPosition()>= Ruth.liftTargetPosition()-150 && !Objects.equals(Ruth.liftLevel(), "home")){
                Ruth.arm("score");
            }

            if(gamepad2.circle){
                Ruth.intake(-1);
            }
            else if(gamepad2.triangle){
                Ruth.lift("home");
                Ruth.arm("safe");
            }


            telemetry.addData("Lift Current Position", Ruth.liftCurrentPosition());
            telemetry.addData("Arm Current Position", Ruth.armCurrentPosition());
            telemetry.addData("Arm Target Position", Ruth.armTargetPosition());
            telemetry.addData("intakeRight", sensorRight.getDistance(DistanceUnit.MM));
            telemetry.addData("intakeLeft", sensorLeft.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
