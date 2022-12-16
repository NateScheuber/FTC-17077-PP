package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.Left_AutoCycle.scoreX;
import static org.firstinspires.ftc.teamcode.Auto.Left_AutoCycle.scoreY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Universal.Ruth_v2_Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "Right")
public class Right_AutoCycle extends LinearOpMode {

    Ruth_v2_Hardware Ruth = new Ruth_v2_Hardware(this);
    public int cycles = 1;

    private DcMotorEx intakeArm   = null;

    public int hue = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Ruth.init();
        intakeArm = hardwareMap.get(DcMotorEx.class, "intakeArm");
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setTargetPosition(0);
        intakeArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeArm.setTargetPositionTolerance(10);
        intakeArm.setPower(1);
        RevColorSensorV3 detectionLeft = hardwareMap.get(RevColorSensorV3.class, "detectionLeft");


        ElapsedTime AutoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        AutoTimer.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "mecanum activated");
        telemetry.update();

        Pose2d startPose = new Pose2d(36, -65.7, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        telemetry.addData("Status", "0");
        telemetry.update();

        Trajectory determine = drive.trajectoryBuilder(startPose,-90)
                .addDisplacementMarker(3,()->{
                    Ruth.claw(0.0);
                })
                .splineToConstantHeading(new Vector2d(35, -42), Math.toRadians(90))
                .addDisplacementMarker( () -> {
                    hue = detectionLeft.green();
                })
                .build();

        Trajectory Preload = drive.trajectoryBuilder(determine.end(), -90)
                .addDisplacementMarker(() -> {
                    Ruth.lift("high");
                    Ruth.arm("back");
                    Ruth.clawFlip(0.2);
                    Ruth.clawRotatePosition(0.75);
                })
                .splineToLinearHeading(new Pose2d(32.5, -7.5, Math.toRadians(-40)), Math.toRadians(110))
                .addDisplacementMarker(() -> {
                    Ruth.claw(0.2);
                })
                .build();
        telemetry.addData("Status", "1");
        telemetry.update();

        Trajectory Grab = drive.trajectoryBuilder(Preload.end())
                .addDisplacementMarker(() -> {
                    Ruth.lift("home");
                    intakeArm.setTargetPosition(175 - cycles * 20);
                    intakeArm.setPower(0.25);
                    Ruth.clawFlip(0.295);
                    Ruth.clawRotatePosition(0.1);
                })
                .splineTo(new Vector2d(48, -13), Math.toRadians(0))
                .splineTo(new Vector2d(55.5, -13), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    Ruth.clawFlip(0);
                })
                .addDisplacementMarker(4, () -> {
                    Ruth.claw(0.2);
                })
                .addDisplacementMarker(20, () -> {
                    Ruth.claw(0.0);
                })
                .build();
        telemetry.addData("Status", "2");
        telemetry.update();


        Trajectory Score = drive.trajectoryBuilder(Grab.end(), true)
                .addDisplacementMarker(() -> {
                    Ruth.lift("high");
                    Ruth.arm("back");
                    Ruth.clawFlip(0.2);
                    Ruth.clawRotatePosition(0.75);
                })
                .splineTo(new Vector2d(48, -13), 180)
                .splineToSplineHeading(new Pose2d(-scoreX, scoreY, Math.toRadians(325)), Math.toRadians(145))
                .build();
        telemetry.addData("Status", "3");
        telemetry.update();


        Trajectory parkZone1 = drive.trajectoryBuilder(Score.end())
                .addDisplacementMarker(() -> {
                    Ruth.lift("home");
                    Ruth.arm("home");
                    Ruth.clawFlip(0.20);
                    Ruth.clawRotatePosition(0.1);
                })
                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)), Math.toRadians(0))
                .build();
        telemetry.addData("Status", "P1");
        telemetry.update();


        Trajectory parkZone2 = drive.trajectoryBuilder(Score.end())
                .addDisplacementMarker(() -> {
                    Ruth.lift("home");
                    Ruth.arm("home");
                    Ruth.clawFlip(0.20);
                    Ruth.clawRotatePosition(0.1);
                })
                .splineToLinearHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .build();
        telemetry.addData("Status", "P2");
        telemetry.update();


        Trajectory parkZone3 = drive.trajectoryBuilder(Score.end())
                .addDisplacementMarker(() -> {
                    Ruth.lift("home");
                    Ruth.arm("home");
                    Ruth.clawFlip(0.20);
                    Ruth.clawRotatePosition(0.1);
                    Ruth.claw(0.4);
                })
                .splineToLinearHeading(new Pose2d(7, -12, Math.toRadians(0)), Math.toRadians(180))
                .build();
        telemetry.addData("Status", "P3");
        telemetry.update();


        telemetry.addData("Status", "Paths Built");
        telemetry.update();

        Trajectory parkTerminal = drive.trajectoryBuilder(Score.end())
                .addDisplacementMarker(()->{
                    Ruth.claw(0.0);
                    Ruth.lift("home");
                    Ruth.arm("home");
                    Ruth.clawFlip(0.24);
                })
                .splineToSplineHeading(new Pose2d(60,-24, Math.toRadians(-90)), Math.toRadians(-90))
                .splineTo(new Vector2d(60,62), Math.toRadians(-90))
                .build();




        //Ruth.initCamera();
        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.setPipeline(new Signal_Pipeline());

        telemetry.addData("Status", "Vision Ready");
        telemetry.update();

        int rotation = 0;
        while (opModeInInit()) {
            rotation = Signal_Pipeline.getAnalysis;
            telemetry.addData("What I See", rotation);
            telemetry.update();
        }
         */


        waitForStart();
        AutoTimer.reset();
        drive.followTrajectory(Preload);


        drive.followTrajectory(Grab);
        drive.followTrajectory(Score);
        cycles = cycles + 1;

        /*
        drive.followTrajectory(Grab);
        drive.followTrajectory(Score);
        cycles = cycles + 1;

        drive.followTrajectory(Grab);
        drive.followTrajectory(Score);
        cycles = cycles + 1;

        drive.followTrajectory(Grab);
        drive.followTrajectory(Score);
        cycles = cycles + 1;



        drive.followTrajectory(parkTerminal);

         */



        if (hue < 800) {
            drive.followTrajectory(parkZone3);
        } else if (hue > 800 && hue < 2300) {
            drive.followTrajectory(parkZone2);
        } else {
            drive.followTrajectory(parkZone1);
        }
    }
}
