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

@Autonomous(name = "BlueAutonRight", group = "Autonomous")
public class BlueAutonRight extends LinearOpMode {
    //kadhir :)

    detectionPipeline pipeline;

    public DcMotorEx liftMotor;
    public DcMotorEx intakeMotor;
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
        outtake.setPower(1);
        Thread.sleep(1000);
        outtake.setPower(0);
    }

    public void runOpMode() throws InterruptedException {
        //Camera initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName OpenCvCamera = hardwareMap.get(WebcamName.class, "frontCamera");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(OpenCvCamera, cameraMonitorViewId);

        pipeline = new detectionPipeline();

        outtake = hardwareMap.crservo.get("outtake");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
        Trajectory forward = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(26, 3))
                .build();

        Trajectory back = drive.trajectoryBuilder(forward.end())
                .lineToConstantHeading(new Vector2d(25, 5))
                .build();

        Trajectory turn = drive.trajectoryBuilder(back.end())
                .lineToLinearHeading(new Pose2d(25, 4, Math.toRadians(91)))
                .build();

        Trajectory forwardToBoard =  drive.trajectoryBuilder(turn.end())
                .lineToConstantHeading(new Vector2d(25, 41))
                .build();

        Trajectory park =  drive.trajectoryBuilder(forwardToBoard.end())
                .lineToConstantHeading(new Vector2d(5, 41))
                .build();

        Trajectory park2 =  drive.trajectoryBuilder(park.end())
                .lineToConstantHeading(new Vector2d(5, 73))
                .build();


        //Position 0
        Trajectory forward0 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(22,15 ))
                .build();

        Trajectory backTurn = drive.trajectoryBuilder(forward0.end())
                .lineToLinearHeading(new Pose2d(25, 4, Math.toRadians(90)))
                .build();

        Trajectory forward02 =  drive.trajectoryBuilder(turn.end())
                .lineToConstantHeading(new Vector2d(25, 45))
                .build();

        Trajectory park0 =  drive.trajectoryBuilder(forward02.end())
                .lineToConstantHeading(new Vector2d(5, 43))
                .build();

        Trajectory park02 =  drive.trajectoryBuilder(park0.end())
                .lineToConstantHeading(new Vector2d(5, 73))
                .build();

        //position2

        Trajectory forward2 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(10,-5 ))
                .build();

        Trajectory backTurn2 = drive.trajectoryBuilder(forward0.end())
                .lineToLinearHeading(new Pose2d(0, -5, Math.toRadians(90)))
                .build();

        Trajectory forward3 =  drive.trajectoryBuilder(backTurn2.end())
                .lineToConstantHeading(new Vector2d(0, 41))
                .build();

        Trajectory right3 =  drive.trajectoryBuilder(backTurn2.end())
                .lineToConstantHeading(new Vector2d(25, 41))
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
                drive.followTrajectory(forward);
                changeLift(1400);
                Thread.sleep(1500);
                outtake();

                drive.followTrajectory(back);
                drive.followTrajectory(turn);
                drive.followTrajectory(forwardToBoard);
                changeLift(2000);
                Thread.sleep(1500);
                outtake();
                changeLift(0);
                drive.followTrajectory(park);
                drive.followTrajectory(park2);


            }

            if (position == 0) {
                drive.followTrajectory(forward0);
                changeLift(1400);
                Thread.sleep(3000);
                outtake();
                drive.followTrajectory(backTurn);
                changeLift(0);

                drive.followTrajectory(forward02);
                changeLift(2000);
                Thread.sleep(1500);
                outtake();

                changeLift(0);
                drive.followTrajectory(park2);
                drive.followTrajectory(park02);

            }

            if (position == 2) {
                drive.followTrajectory(forward2);
                changeLift(1400);
                Thread.sleep(3000);
                outtake();
                drive.followTrajectory(backTurn2);
                changeLift(0);

                drive.followTrajectory(forward3);
                changeLift(2000);
                Thread.sleep(1500);
                outtake();

                changeLift(0);
                drive.followTrajectory(park0);
                drive.followTrajectory(park02);
            }





        }

    }
}
