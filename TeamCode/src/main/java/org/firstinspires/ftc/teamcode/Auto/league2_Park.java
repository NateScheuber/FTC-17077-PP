package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Universal.Ruth_v2_Hardware;

@Autonomous (name = "League 2 Auto")
public class league2_Park extends LinearOpMode {
    Ruth_v2_Hardware Ruth = new Ruth_v2_Hardware(this);



    @Override
    public void runOpMode() throws InterruptedException {
        Ruth.init();

        waitForStart();
        Ruth.claw(0.0);
        sleep(200);
        Ruth.driveRobotOriented(0,0.5,0);
        sleep(600);
        Ruth.driveRobotOriented(0,-0.5,0);
        sleep(100);
        Ruth.driveRobotOriented(0,0,0);
    }
}
