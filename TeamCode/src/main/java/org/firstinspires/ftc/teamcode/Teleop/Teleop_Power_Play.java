package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Universal.RuthHardware;

import java.util.List;


public class Teleop_Power_Play extends LinearOpMode {

    private double driveY = 0;
    private double driveX = 0;
    private double driveR = 0;
    private double heading =0;


    Orientation angles;

    private BNO055IMU imu = null;


    RuthHardware Ruth = new RuthHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        Ruth.init();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        waitForStart();
        while(opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            driveX = gamepad1.right_stick_x;
            driveY = -gamepad1.right_stick_y;
            driveR = gamepad1.left_stick_x;
            heading = angles.firstAngle;


            //dt control
            Ruth.driveRobotOriented(driveX, driveY, driveR);

            //intake control
            if(gamepad1.right_bumper && Ruth.intakeSensor()){
                Ruth.intake("in");
            }
            else if(gamepad1.left_bumper){
                Ruth.intake("out");
            }


            //lift control
            if(gamepad2.dpad_down){
                Ruth.lift("home");
            }
            else if(gamepad2.dpad_left){
                Ruth.lift("low");
            }
            else if(gamepad2.dpad_right){
                Ruth.lift("medium");
            }
            else if(gamepad2.dpad_up){
                Ruth.lift("high");
            }

            //score
            if(gamepad2.cross){
                Ruth.score();
            }





            Ruth.lift("intakeDown");

        }
    }
}
