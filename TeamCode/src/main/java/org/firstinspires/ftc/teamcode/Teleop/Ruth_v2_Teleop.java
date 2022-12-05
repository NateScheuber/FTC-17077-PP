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

    int armOffset = 0;

    boolean clawClosed = false;
    boolean clawClosedToggle = false;

    boolean clawFlipped = false;
    boolean clawFlippedToggle = false;

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


            //Ruth.clawProtection();
            Ruth.clawRotate();



            if(gamepad2.dpad_down){
                Ruth.lift("home");
                Ruth.arm("home");
                Ruth.clawFlip(0.24);
                liftPosition = 0;
            }
            else if(gamepad2.dpad_left){
                Ruth.lift("home");
                Ruth.arm("up");
                Ruth.clawFlip(0.36);
                liftPosition = 1;
            }
            else if(gamepad2.dpad_up){
                Ruth.lift("medium");
                Ruth.arm("back");
                Ruth.clawFlip(0.2);
                liftPosition = 2;
            }
            else if(gamepad2.dpad_right){
                Ruth.lift("high");
                Ruth.arm("back");
                Ruth.clawFlip(0.2);
                liftPosition = 3;
            }

            //claw grab control
            if((gamepad1.right_bumper || gamepad2.circle) && clawClosedToggle){
                clawClosedToggle = false;
                if(clawClosed){
                    clawClosed=false;
                    Ruth.claw(0.2);
                }
                else{
                    clawClosed=true;
                    Ruth.claw(0.0);
                }
            }
            else if(!gamepad1.right_bumper && !clawClosedToggle){
                clawClosedToggle = true;
            }

            //claw flip control
            if(gamepad1.left_bumper && clawFlippedToggle){
                clawFlippedToggle = false;
                if(clawFlipped){
                    clawFlipped = false;
                    Ruth.clawFlip(0.24);
                }
                else if(clawClosed){
                    clawFlipped = true;
                    Ruth.clawFlip(0.0);
                }
                else{
                    clawFlipped = false;
                    Ruth.clawFlip(0.24);
                }
            }
            else if(!gamepad1.left_bumper && !clawFlippedToggle){
                clawFlippedToggle = true;
            }

            //arm offset
            if(gamepad2.right_bumper){
                armOffset += 1;
            }
            else if(gamepad2.left_bumper){
                armOffset -= 1;
            }

            //full reset
            if(gamepad2.triangle){
                Ruth.lift("home");
                Ruth.arm("home");
                Ruth.claw(0.0);
                Ruth.clawFlip(0.24);
            }

            telemetry.addData("Lift Position", Ruth.liftCurrentPosition());
            telemetry.addData("Lift Target Position", Ruth.liftTargetPosition());
            telemetry.addData("Arm Position", Ruth.armCurrentPosition());
            telemetry.addData("Claw Closed?", clawClosed);
            telemetry.addData("Claw Flipped?", clawFlipped);
            telemetry.update();

        }
    }
}
