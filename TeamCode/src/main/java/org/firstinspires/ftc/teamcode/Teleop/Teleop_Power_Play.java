package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
            driveR = gamepad1.left_stick_x+(gamepad1.right_trigger*0.25)-(gamepad1.left_trigger*0.25);
            heading = angles.firstAngle;


            //dt control
            Ruth.driveRobotOriented(driveX, driveY, driveR);
            Ruth.driveSlowMo(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left);

            //intake control
            if(gamepad1.right_bumper || gamepad1.left_bumper) {
                Ruth.pickUp(gamepad1.right_bumper, gamepad1.left_bumper);
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
                Ruth.arm("score");
            }
            else if(gamepad2.triangle){
                Ruth.score();
            }
        }
    }
}
