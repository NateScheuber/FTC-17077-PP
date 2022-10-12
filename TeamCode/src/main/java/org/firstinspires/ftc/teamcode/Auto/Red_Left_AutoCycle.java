package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Universal.RuthHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Red_Left_AutoCycle extends LinearOpMode {

    public enum path{
        preload,
        grab,
        score,
        park,
        IDLE
    }

    RuthHardware Ruth = new RuthHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        Ruth.init();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        Trajectory Preload = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-62,-52), Math.toRadians(90))
                .splineTo(new Vector2d(-57, -19), Math.toRadians(45))

                .addDisplacementMarker(() -> {
                    Ruth.lift("high");
                    Ruth.arm("score");
                })

                .splineToSplineHeading(new Pose2d(-32,-8, Math.toRadians(-135)), Math.toRadians(25))
                .addDisplacementMarker(() -> {
                    Ruth.score();
                })
                .build();

        Trajectory Grab = drive.trajectoryBuilder(Preload.end())
                .splineToSplineHeading(new Pose2d(-48,-12, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-62, -12), Math.toRadians(180))
                .build();

        Trajectory Score = drive.trajectoryBuilder(Grab.end())
                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-32, -8, Math.toRadians(-135)), Math.toRadians(40))
                .build();

        Trajectory parkZone1 = drive.trajectoryBuilder(Score.end())
                .splineToSplineHeading(new Pose2d(-60, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-60, -36), Math.toRadians(-90))
                .build();

        Trajectory parkZone2 = drive.trajectoryBuilder(Score.end())
                .splineToLinearHeading(new Pose2d(-36, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        Trajectory parkZone3 = drive.trajectoryBuilder(Score.end())
                .splineTo(new Vector2d(-12, -24), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-12, -36, Math.toRadians(-90)), Math.toRadians(-90))
                .build();
        telemetry.addData("Status", "Paths Built");
        telemetry.update();



        telemetry.addData("Status", "Vision Ready");
        telemetry.update();


        while(opModeInInit()){
            //vision code
        }


        waitForStart();



    }
}
