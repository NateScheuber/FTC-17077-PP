package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

    @Override
    public void runOpMode() throws InterruptedException {
        Ruth.init();
        intakeArm = hardwareMap.get(DcMotorEx.class, "intakeArm");
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setTargetPosition(0);
        intakeArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeArm.setTargetPositionTolerance(10);
        intakeArm.setPower(1);

        ElapsedTime AutoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        AutoTimer.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("Status", "mecanum activated");
        telemetry.update();

        Pose2d startPose = new Pose2d(36, -65.7, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        telemetry.addData("Status", "0");
        telemetry.update();

        Trajectory Preload = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(60, -48), Math.toRadians(150))
                .splineTo(new Vector2d(48, -14), Math.toRadians(160))
                .splineToSplineHeading(new Pose2d(32, -8, Math.toRadians(160)), Math.toRadians(-45))
                .addDisplacementMarker(()->{
                    Ruth.claw(0.2);
                })
                .addDisplacementMarker(25,()->{
                    Ruth.lift("high");
                    Ruth.arm("back");
                    Ruth.clawFlip(0.2);
                })
                .build();
        telemetry.addData("Status", "1");
        telemetry.update();

        Trajectory Grab = drive.trajectoryBuilder(Preload.end())
                .addDisplacementMarker(()->{
                    Ruth.claw(0);
                    Ruth.lift("home");
                    intakeArm.setTargetPosition(50-cycles*10);
                    Ruth.clawFlip(0.24);
                })
                .splineTo(new Vector2d(48, -12), Math.toRadians(0))
                .addDisplacementMarker(()->{
                    Ruth.claw(0.2);
                })
                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                .addDisplacementMarker(()->{
                    Ruth.claw(0.0);
                })
                .build();
        telemetry.addData("Status", "2");
        telemetry.update();


        Trajectory Score = drive.trajectoryBuilder(Grab.end())
                .addDisplacementMarker(()->{
                    Ruth.clawFlip(0.0);
                })
                .splineTo(new Vector2d(48, -12), 180)
                .addDisplacementMarker(25,()->{
                    Ruth.lift("high");
                    Ruth.arm("back");
                    Ruth.clawFlip(0.2);
                })
                .splineToSplineHeading(new Pose2d(32, -8, Math.toRadians(135)), Math.toRadians(-45))
                .addDisplacementMarker(()->{
                    Ruth.claw(0.2);
                })
                .build();
        telemetry.addData("Status", "3");
        telemetry.update();


        Trajectory parkZone1 = drive.trajectoryBuilder(Score.end())
                .splineToSplineHeading(new Pose2d(60, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(60, -36), Math.toRadians(-90))
                .build();
        telemetry.addData("Status", "P1");
        telemetry.update();


        Trajectory parkZone2 = drive.trajectoryBuilder(Score.end())
                .splineToLinearHeading(new Pose2d(36, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .build();
        telemetry.addData("Status", "P2");
        telemetry.update();


        Trajectory parkZone3 = drive.trajectoryBuilder(Score.end())
                .splineTo(new Vector2d(12, -24), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(12, -36, Math.toRadians(-90)), Math.toRadians(-90))
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


        /*
        if (rotation == 1){
            drive.followTrajectory(parkZone1);
        }
        else if(rotation ==2) {
            drive.followTrajectory(parkZone2);
        }
        else{
            drive.followTrajectory(parkZone3);
        }\
         */
    }
}
