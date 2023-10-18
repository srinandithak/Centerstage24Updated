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
    public CRServo outtake;
    public Servo droneLauncher;
    public static final int BOTTOM_LEVEL_POSITION = 1800;
    public static final int MIDDLE_LEVEL_POSITION = 2950;
    public static final int TOP_LEVEL_POSITION = 4000;
    public static final int TOP_LEVEL = 3;
    public static final int MIDDLE_LEVEL = 2;
    public static final int BOTTOM_LEVEL = 1;
    public int detectedLevel;

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

        int position = pipeline.getAnalysis();

        new detectionPipeline();
        telemetry.addData("object detection", position);

        telemetry.update();


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);



        //Position
        Trajectory forward = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(30, -5))
                .build();

        Trajectory back = drive.trajectoryBuilder(forward.end())
                .lineToConstantHeading(new Vector2d(25, -5))
                .build();

        Trajectory turn = drive.trajectoryBuilder(back.end())
                .lineToLinearHeading(new Pose2d(25, -4, Math.toRadians(85)))
                .build();

        Trajectory forward2 =  drive.trajectoryBuilder(turn.end())
           .lineToConstantHeading(new Vector2d(30, 65))
           .build();

        Trajectory park =  drive.trajectoryBuilder(forward2.end())
                .lineToConstantHeading(new Vector2d(54, 55))
                .build();

        Trajectory park2 =  drive.trajectoryBuilder(park.end())
                .lineToConstantHeading(new Vector2d(54, 65))
                .build();



//        Trajectory dropBlock = drive.trajectoryBuilder(strafeRight.end())
//                .lineToLinearHeading(new Pose2d(46.5, -16.6, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        Trajectory goBack1 = drive.trajectoryBuilder(dropBlock.end())
//                .back(3.5, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        Trajectory strafeRight2 = drive.trajectoryBuilder(goBack1.end())
//                .strafeRight(14)
//                .build();
//
//        Trajectory pickBlock = drive.trajectoryBuilder(strafeRight2.end())
//                .lineToLinearHeading(new Pose2d(62, 32.15, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        Trajectory goBack = drive.trajectoryBuilder(pickBlock.end())
//                .back(25, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        Trajectory dropBlock2 = drive.trajectoryBuilder(goBack.end())
//                .lineToLinearHeading(new Pose2d(58.1, 16, Math.toRadians(180)))
//                .build();
//
//        Trajectory goBack2 = drive.trajectoryBuilder(dropBlock2.end())
//                .back(9, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        Trajectory pickBlock2 = drive.trajectoryBuilder(goBack2.end())
//                .lineToLinearHeading(new Pose2d(62, 32.15, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        Trajectory reset = drive.trajectoryBuilder(dropBlock2.end())
//                .lineToLinearHeading(new Pose2d(64, 27, Math.toRadians(184)))
//                .build();


        waitForStart();
        if (isStopRequested()) {
            return;
        }

        if (opModeIsActive()) {
            drive.followTrajectory(forward);
            changeLift(1000);
            outtake();
            drive.followTrajectory(back);
            drive.followTrajectory(turn);
            drive.followTrajectory(forward2);
            changeLift(2000);
            outtake();
            drive.followTrajectory(park);
            drive.followTrajectory(park2);


        }

    }
}
