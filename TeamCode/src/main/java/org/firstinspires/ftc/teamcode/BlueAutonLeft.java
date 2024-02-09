package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.TeleOp.NORMAL_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.List;
import java.util.Vector;

@Autonomous(name = "BlueAutonLeft", group = "Autonomous")
public class BlueAutonLeft extends LinearOpMode {

    detectionPipeline pipeline;

    public DcMotorEx liftMotor;
    public DcMotorEx intakeMotor;

    public Servo leftRampServo;

    public CRServo intakeServo;
    public CRServo outtake;
    public Servo droneLauncher;

    public Servo stackServo;
    public DistanceSensor distanceSensorLeft;

    public DistanceSensor distanceSensorRight;
    public double robotSpeed = NORMAL_SPEED;
    public double rotationSpeed = 1;

    SampleMecanumDrive drive;



    public void distanceSensor() {
        if (gamepad1.dpad_left || gamepad2.dpad_left) {

            if (distanceSensorLeft.getDistance(DistanceUnit.INCH) > distanceSensorRight.getDistance(DistanceUnit.INCH)) {
                while(distanceSensorLeft.getDistance(DistanceUnit.INCH) >= distanceSensorRight.getDistance(DistanceUnit.INCH))
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * robotSpeed,
                                    -gamepad1.left_stick_x * robotSpeed,
                                    -1
                            )
                    );
            }

            if (distanceSensorRight.getDistance(DistanceUnit.INCH) > distanceSensorLeft.getDistance(DistanceUnit.INCH)) {
                while(distanceSensorRight.getDistance(DistanceUnit.INCH) >= distanceSensorLeft.getDistance(DistanceUnit.INCH))
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * robotSpeed,
                                    -gamepad1.left_stick_x * robotSpeed,
                                    1
                            )
                    );
            }

            if (distanceSensorLeft.getDistance(DistanceUnit.INCH) < 7.559) {
                while (distanceSensorLeft.getDistance(DistanceUnit.INCH) <= 7.559) {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * robotSpeed,
                                    -gamepad1.left_stick_x * robotSpeed,
                                    -gamepad1.right_stick_x * robotSpeed * rotationSpeed
                            )
                    );
                }
            } else if (distanceSensorLeft.getDistance(DistanceUnit.INCH) > 7.559) {
                while (distanceSensorLeft.getDistance(DistanceUnit.INCH) >= 7.559) {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * robotSpeed,
                                    -gamepad1.left_stick_x * robotSpeed,
                                    -gamepad1.right_stick_x * robotSpeed * rotationSpeed
                            )
                    );
                }
            }
        }
    }

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
        Thread.sleep(300);
        outtake.setPower(0);
        intakeServo.setPower(0);
    }

    public void outtakeGround() throws InterruptedException {
        intakeMotor.setPower(.28);
        Thread.sleep(1000);
        intakeMotor.setPower(0);
    }

    public void pixel() throws InterruptedException {
        intakeMotor.setPower(1);
        intakeServo.setPower(-1);
        Thread.sleep(2000);
        intakeMotor.setPower(0);
        intakeServo.setPower(0);
    }

    public void outtakePos() throws InterruptedException {
        leftRampServo.setPosition(0.73);
    }

    public void intakePos() throws InterruptedException {

        leftRampServo.setPosition(0.20);

    }

    public void stackServo(double position, Pose2d initialPos) throws InterruptedException {


        Trajectory pixelStack = drive.trajectoryBuilder(initialPos)
                .lineToConstantHeading(new Vector2d(-55,58))
                .build();

        Trajectory pickUp = drive.trajectoryBuilder(pixelStack.end())
                .lineToConstantHeading(new Vector2d(-53,68))
                .build();

        Trajectory pixelStack2 = drive.trajectoryBuilder(pickUp.end())
                .lineToLinearHeading(initialPos)
                .build();

        stackServo.setPosition(position);
        drive.followTrajectory(pixelStack);
        intakeMotor.setPower(-1);
        intakeServo.setPower(1);
        stackServo.setPosition(1);
        drive.followTrajectory(pickUp);
        Thread.sleep(1000);
        intakeMotor.setPower(0);
        intakeServo.setPower(0);
        drive.followTrajectory(pixelStack2);

    }
    public void runOpMode() throws InterruptedException {
        //Camera initialization Kadhir was Here :)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName OpenCvCamera = hardwareMap.get(WebcamName.class, "frontCamera");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(OpenCvCamera, cameraMonitorViewId);

        pipeline = new detectionPipeline();

        outtake = hardwareMap.crservo.get("outtake");
        intakeServo = hardwareMap.crservo.get("intakeServo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        leftRampServo = hardwareMap.get(Servo.class, "leftRampServo");
//        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
//        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        stackServo = hardwareMap.get(Servo.class, "stackServo");



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




        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);


        int position = pipeline.getAnalysis();

        Trajectory strafe = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-3, -12))
                .build();



        //POSITION 0

        Trajectory dropPos_0 = drive.trajectoryBuilder(strafe.end())
                .lineToConstantHeading(new Vector2d(-25,-14))
                .build();
        Trajectory backBoard_0 = drive.trajectoryBuilder(dropPos_0.end())
                .lineToLinearHeading(new Pose2d(-18,-43, Math.toRadians(-90)))
                .build();
        Trajectory reset_0 = drive.trajectoryBuilder(backBoard_0.end())
                .lineToLinearHeading(new Pose2d(-24,-35, Math.toRadians(-90)))
                .build();



        //POSITION 1

        Trajectory dropPos_1 = drive.trajectoryBuilder(strafe.end())
                .lineToConstantHeading(new Vector2d(-27,-4))
                .build();
        Trajectory backBoard_1 = drive.trajectoryBuilder(dropPos_1.end())
                .lineToLinearHeading(new Pose2d(-24,-42, Math.toRadians(-90)))
                .build();
        Trajectory reset_1 = drive.trajectoryBuilder(backBoard_1.end())
                .lineToLinearHeading(new Pose2d(-24,-35, Math.toRadians(-90)))
                .build();

        //POSITION 2

        Trajectory dropPos_2 = drive.trajectoryBuilder(strafe.end())
                .lineToLinearHeading(new Pose2d(-26, 0, Math.toRadians(-90)))
                .build();

        Trajectory backBoard_2 = drive.trajectoryBuilder(dropPos_2.end())
                .lineToConstantHeading(new Vector2d(-31, -43))
                .build();
        Trajectory reset_2 = drive.trajectoryBuilder(backBoard_2.end())
                .lineToLinearHeading(new Pose2d(-24,-35, Math.toRadians(-90)))
                .build();

        //GO TO PIXELS
        Trajectory shift = drive.trajectoryBuilder(reset_0.end())
                .lineToConstantHeading(new Vector2d(-52, -35))
                .build();

        Trajectory pixels =  drive.trajectoryBuilder(shift.end())
                .lineToConstantHeading(new Vector2d(-55, 68))
                .build();

        Trajectory back = drive.trajectoryBuilder(pixels.end())
                .lineToConstantHeading(new Vector2d(-52, -35))
                .build();

        Trajectory backBoard = drive.trajectoryBuilder(back.end())
                .lineToLinearHeading(new Pose2d(-22,-41.5, Math.toRadians(-90)))
                .build();

        Trajectory park =  drive.trajectoryBuilder(reset_0.end())
                .lineToConstantHeading(new Vector2d(3, -20))
                .build();

        Trajectory park2 = drive.trajectoryBuilder(park.end())
                .lineToConstantHeading(new Vector2d(3, -50))
                .build();

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
            //Position 0

//            intakePos();
            leftRampServo.setPosition(.25);
            changeLift(80);
            drive.followTrajectory(strafe);

            if (position == 0) {
                drive.followTrajectory(dropPos_0);
                outtakeGround();
                changeLift(1900);
                drive.followTrajectory(backBoard_0);
                outtakePos();
                Thread.sleep(500);
                outtake();
                drive.followTrajectory(reset_0);
            }
            else if (position == 1) {
                drive.followTrajectory(dropPos_1);
                outtakeGround();
                changeLift(1900);
                drive.followTrajectory(backBoard_1);
                outtakePos();
                Thread.sleep(500);
                outtake();
                drive.followTrajectory(reset_1);

            }
            else {
                drive.followTrajectory(dropPos_2);
                outtakeGround();
                changeLift(1900);
                drive.followTrajectory(backBoard_2);
                outtakePos();
                Thread.sleep(500);
                outtake();
                drive.followTrajectory(reset_2);
            }

            intakePos();
            changeLift(0);
            leftRampServo.setPosition(.25);
            drive.followTrajectory(shift);
//            stackServo.setPosition(.2);
            drive.followTrajectory(pixels);
            intakePos();
//            stackServo(0, pixels.end());
            intakeMotor.setPower(-1);
            intakeServo.setPower(1);
            Thread.sleep(3000);
            intakeMotor.setPower(0);
            intakeServo.setPower(0);
            drive.followTrajectory(back);
            drive.followTrajectory(backBoard);
            changeLift(1900);
            Thread.sleep(1000);
            outtakePos();
            Thread.sleep(500);
            outtake();
            drive.followTrajectory(park);
            intakePos();
            changeLift(0);
            drive.followTrajectory(park2);




        }

    }
}
