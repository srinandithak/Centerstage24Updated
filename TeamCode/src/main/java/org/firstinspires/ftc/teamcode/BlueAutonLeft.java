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

@Autonomous(name = "BlueAutonLeft", group = "Autonomous")
public class BlueAutonLeft extends LinearOpMode {

    detectionPipeline pipeline;

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
        //Camera initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName OpenCvCamera = hardwareMap.get(WebcamName.class, "frontCamera");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(OpenCvCamera, cameraMonitorViewId);

        pipeline = new detectionPipeline();

        outtake = hardwareMap.crservo.get("outtake");
        intakeServo = hardwareMap.crservo.get("intakeServo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
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
                .lineToConstantHeading(new Vector2d(45, 0))
                .build();

//        Trajectory forward1_2 = drive.trajectoryBuilder(forward1.end())
//                .lineToConstantHeading(new Vector2d(55, 0))
//                .build();

        Trajectory backTurn1 = drive.trajectoryBuilder(forward1.end())
                .lineToLinearHeading(new Pose2d(19, 0, Math.toRadians(93)))
                .build();


        Trajectory dropPos1 = drive.trajectoryBuilder(backTurn1.end())
                .lineToConstantHeading(new Vector2d(19, 44))
                .build();

        Trajectory park1 = drive.trajectoryBuilder(dropPos1.end())
                .lineToConstantHeading(new Vector2d(19, 25))
                .build();

        Trajectory park1_2 = drive.trajectoryBuilder(park1.end())
                .lineToConstantHeading(new Vector2d(-31, 25))
                .build();

        Trajectory park1_3 = drive.trajectoryBuilder(park1_2.end())
                .lineToConstantHeading(new Vector2d(-31, 45))
                .build();





        //Position 0
        Trajectory forward0 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(24,0 ))
                .build();

        Trajectory dropPos0 = drive.trajectoryBuilder(forward0.end())
                .lineToConstantHeading(new Vector2d(24,24 ))
                .build();

        Trajectory backBoard0 = drive.trajectoryBuilder(dropPos0.end())
                .lineToConstantHeading(new Vector2d(20,40 ))
                .build();


        Trajectory park0 =  drive.trajectoryBuilder(backBoard0.end())
                .lineToConstantHeading(new Vector2d(20, 30))
                .build();

        Trajectory park0_2 = drive.trajectoryBuilder(park0.end())
                .lineToConstantHeading(new Vector2d(-31, 30))
                .build();

        Trajectory park0_3 = drive.trajectoryBuilder(park0_2.end())
                .lineToConstantHeading(new Vector2d(-31, 42))
                .build();


        //position2

        Trajectory forward2 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(24,0 ))
                .build();

        Trajectory dropPos2 = drive.trajectoryBuilder((forward2.end()))
                .lineToConstantHeading(new Vector2d(24,-5))
                .build();

        Trajectory backBoard2 = drive.trajectoryBuilder(dropPos2.end())
                .lineToConstantHeading(new Vector2d(30,40))
                .build();

        Trajectory park2 =  drive.trajectoryBuilder(backBoard2.end())
                .lineToConstantHeading(new Vector2d(30, 30))
                .build();

        Trajectory park2_2 =  drive.trajectoryBuilder(park2.end())
                .lineToConstantHeading(new Vector2d(-31, 30))
                .build();

        Trajectory park2_3 =  drive.trajectoryBuilder(park2_2.end())
                .lineToConstantHeading(new Vector2d(-31, 42))
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
                Thread.sleep(1000);
                changeLift(0);
                //drive.followTrajectory(forward1_2);
                drive.followTrajectory(backTurn1);
                //goes to backboard
                drive.followTrajectory(dropPos1);
                changeLift(1650);
                outtakePos();
                Thread.sleep(1000);
                outtake();
                drive.followTrajectory(park1);
                intakePos();
                changeLift(0);
                drive.followTrajectory(park1_2);
                drive.followTrajectory(park1_3);
            }

            if (position == 0) {
                drive.followTrajectory(forward0);
                drive.turn(Math.toRadians(93));
                drive.followTrajectory(dropPos0);
                pixel();
                drive.followTrajectory(backBoard0);
                changeLift(1650);
                //rampPos();
                outtake();
                drive.followTrajectory(park0);
                drive.followTrajectory(park0_2);
                drive.followTrajectory(park0_3);
                changeLift(0);
                //rampPos();


            }

            if (position == 2) {
                drive.followTrajectory(forward2);
                drive.turn(Math.toRadians(93));
                drive.followTrajectory(dropPos2);
                pixel();
                drive.followTrajectory(backBoard2);
                changeLift(1650);
                //rampPos();
                outtake();
                drive.followTrajectory(park2);
                drive.followTrajectory(park2_2);
                drive.followTrajectory(park2_3);
                changeLift(0);
                //rampPos();

            }


        }

    }
}
