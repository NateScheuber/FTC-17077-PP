package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Universal.Ruth_v2_Hardware;

//@Config
@TeleOp (name = "v2 Teleop")
public class Ruth_v2_Teleop extends LinearOpMode {

    int liftPosition = 0;

    Orientation angles;
    private BNO055IMU imu = null;

    Ruth_v2_Hardware Ruth = new Ruth_v2_Hardware(this);


    @Override
    public void runOpMode() throws InterruptedException {
        Ruth.init();


        waitForStart();
        while(opModeIsActive()){
            double driveX = gamepad1.right_stick_x;
            double driveY = -gamepad1.right_stick_y;
            double driveR = gamepad1.left_stick_x + (gamepad1.right_trigger * 0.25) - (gamepad1.left_trigger * 0.25);

            //DT Control
            Ruth.driveRobotOriented(driveX,driveY,driveR);
            Ruth.driveSlowMo(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left);

            Ruth.claw(gamepad1.right_bumper);
            Ruth.clawProtection();
            Ruth.clawRotate();
            Ruth.clawFlip(gamepad1.left_bumper);

            if(gamepad2.dpad_down){
                Ruth.lift("home");

                liftPosition = 0;
            }
            else if(gamepad2.dpad_left){
                Ruth.lift("low");
                liftPosition = 1;
            }
            else if(gamepad2.dpad_right){
                Ruth.lift("high");
                liftPosition = 2;
            }
            else if(gamepad2.dpad_up){
                Ruth.lift("medium");
                liftPosition = 3;
            }


            if(gamepad2.triangle){
                Ruth.lift("home");
                Ruth.arm("home");
            }

            telemetry.addData("Lift Position", Ruth.liftCurrentPosition());
            telemetry.update();

        }
    }
}
