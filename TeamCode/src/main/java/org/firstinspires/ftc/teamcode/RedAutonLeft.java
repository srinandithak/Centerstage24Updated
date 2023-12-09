package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.List;

@Autonomous(name = "RedAutonleft", group = "Autonomous")
public class RedAutonLeft extends LinearOpMode {

    detectionPipelineRed pipeline;

    public DcMotorEx liftMotor;
    public DcMotorEx intakeMotor;

    public Servo rightRampServo;

    public CRServo intakeServo;
    public CRServo outtake;
    public Servo droneLauncher;

    public void changeLift (int height)  throws InterruptedException {
        Thread.sleep(100);


        if (liftMotor.getCurrentPosition() < height) {
            liftMotor.setTargetPosition(height);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.9);
        }
        else {
            liftMotor.setTargetPosition(height);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(-0.7);
        }
    }

    public void outtake() throws InterruptedException {
        outtake.setPower(-1);
        intakeServo.setPower(1);
        Thread.sleep(2250);
        outtake.setPower(0);
        intakeServo.setPower(0);
    }

    public void outtakeGround() throws InterruptedException {
        intakeServo.setPower(-1);
        Thread.sleep(2250);
        intakeServo.setPower(0);
    }

    public void pixel() throws InterruptedException {
        intakeMotor.setPower(1);
        intakeServo.setPower(-1);
        Thread.sleep(2000);
        intakeMotor.setPower(0);
        intakeServo.setPower(0);
    }

    public void outtakePos() throws InterruptedException {

        rightRampServo.setPosition(0.8);


    }

    public void intakePos() throws InterruptedException {

        rightRampServo.setPosition(0.15);

    }

    public void runOpMode() throws InterruptedException {
        //Camera initialization Kadhir was Here :)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName OpenCvCamera = hardwareMap.get(WebcamName.class, "frontCamera");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(OpenCvCamera, cameraMonitorViewId);

        pipeline = new detectionPipelineRed();

        outtake = hardwareMap.crservo.get("outtake");
        intakeServo = hardwareMap.crservo.get("intakeServo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        rightRampServo = hardwareMap.get(Servo.class, "rightRampServo");
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightRampServo.setDirection(rightRampServo.getDirection().REVERSE);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);



        //Position 1
        Trajectory forward1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(44,0 ))
                .build();

        Trajectory back1 = drive.trajectoryBuilder(forward1.end())
                .lineToConstantHeading(new Vector2d(20,0 ))
                .build();




        //Position 2
        Trajectory forward2 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(13,0 ))
                .build();

        Trajectory dropPos2 = drive.trajectoryBuilder((forward2.end()))
                .lineToLinearHeading(new Pose2d(13, -1, Math.toRadians(-91)))
                .build();

        Trajectory dropPos2_2 = drive.trajectoryBuilder(dropPos2.end())
                .lineToConstantHeading(new Vector2d(13, -17 ))
                .build();

        Trajectory back2 = drive.trajectoryBuilder(dropPos2_2.end())
                .lineToConstantHeading(new Vector2d(13, 0 ))
                .build();


        //position0

        Trajectory forward0 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(29,0 ))
                .build();

        Trajectory dropPos0  = drive.trajectoryBuilder(forward0.end())
                .lineToLinearHeading(new Pose2d(29, -1, Math.toRadians(91)))
                .build();

        Trajectory dropPos0_2 = drive.trajectoryBuilder(dropPos0.end())
                .lineToConstantHeading(new Vector2d(32,12))
                .build();

        Trajectory back0 = drive.trajectoryBuilder(dropPos0_2.end())
                .lineToConstantHeading(new Vector2d(31,0 ))
                .build();


        int position = pipeline.getAnalysis();


        while (!opModeIsActive()) {
            position = pipeline.getAnalysis();
            telemetry.addData("object detection", position);
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();
            if (isStopRequested()) {
                return;
            }
        }

        waitForStart();


        if (opModeIsActive()) {
            if (position == 1) {
                drive.followTrajectory(forward1);
                changeLift(500);
                outtakeGround();
                drive.followTrajectory(back1);
                changeLift(0);
                Thread.sleep(1000);


            }

            if (position == 0) {
                drive.followTrajectory(forward0);
                drive.followTrajectory(dropPos0);
                drive.followTrajectory(dropPos0_2);
                changeLift(500);
                outtakeGround();
                drive.followTrajectory(back0);
                changeLift(0);
                Thread.sleep(1000);




            }

            if (position == 2) {
                drive.followTrajectory(forward2);
                drive.followTrajectory(dropPos2);
                drive.followTrajectory(dropPos2_2);
                changeLift(600);
                outtakeGround();
                drive.followTrajectory(back2);
                changeLift(0);
                Thread.sleep(1000);


            }


        }

    }
}
