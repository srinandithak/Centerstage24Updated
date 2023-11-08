package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueAutonLeft2", group = "Autonomous")
public class BlueAutonLeft2 extends LinearOpMode {

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
        Thread.sleep(2000);
        outtake.setPower(0);
    }

    public void outtake2() throws InterruptedException {
        intakeMotor.setPower(0.4);
        Thread.sleep(3000);
        intakeMotor.setPower(0);
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
                .lineToConstantHeading(new Vector2d(24, 3))
                .build();

        Trajectory turn = drive.trajectoryBuilder(forward.end())
                .lineToLinearHeading(new Pose2d(26, 4, Math.toRadians(178)))
                .build();

        Trajectory turn2 = drive.trajectoryBuilder(turn.end())
                .lineToLinearHeading(new Pose2d(26, 15, Math.toRadians(90)))
                .build();

        Trajectory forwardToBoard =  drive.trajectoryBuilder(turn2.end())
           .lineToConstantHeading(new Vector2d(32, 35))
           .build();

        Trajectory park =  drive.trajectoryBuilder(forwardToBoard.end())
                .lineToConstantHeading(new Vector2d(-5, 35))
                .build();

        Trajectory park2 =  drive.trajectoryBuilder(park.end())
                .lineToConstantHeading(new Vector2d(-5, 53))
                .build();


        //Position 0
        Trajectory forward0 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(15,18 ))
                .build();

        Trajectory turn0 = drive.trajectoryBuilder(forward.end())
                .lineToLinearHeading(new Pose2d(19, 18, Math.toRadians(180)))
                .build();

        Trajectory backTurn = drive.trajectoryBuilder(turn0.end())
                .lineToLinearHeading(new Pose2d(15, 4, Math.toRadians(180)))
                .build();

        Trajectory forward02 =  drive.trajectoryBuilder(backTurn.end())
                .lineToLinearHeading(new Pose2d(32.5, 48, Math.toRadians(90)))
                .build();

        Trajectory park0 =  drive.trajectoryBuilder(forward02.end())
                .lineToConstantHeading(new Vector2d(0, 35))
                .build();

        Trajectory park02 =  drive.trajectoryBuilder(park0.end())
                .lineToConstantHeading(new Vector2d(0, 57
                ))
                .build();

        //position2

        Trajectory forward3 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(27,10 ))
                .build();

        Trajectory forward3_2 = drive.trajectoryBuilder(forward3.end())
                .lineToLinearHeading(new Pose2d(27,-2, Math.toRadians(90) ))
                .build();

        Trajectory forward03 =  drive.trajectoryBuilder(forward3_2.end())
                .lineToConstantHeading(new Vector2d(50, 48))
                .build();

        Trajectory park03 =  drive.trajectoryBuilder(forward03.end())
                .lineToConstantHeading(new Vector2d(-8, 35))
                .build();

        Trajectory park03_2 =  drive.trajectoryBuilder(park03.end())
                .lineToConstantHeading(new Vector2d(-8, 53))
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
                drive.followTrajectory(turn);
                outtake2();
                drive.followTrajectory(forwardToBoard);
                changeLift(2500);
                Thread.sleep(1500);
                outtake();
                changeLift(0);
                drive.followTrajectory(park);
                drive.followTrajectory(park2);


            }

            if (position == 0) {
                drive.followTrajectory(forward0);
                drive.followTrajectory(turn0);
                outtake2();
                drive.followTrajectory(backTurn);

                drive.followTrajectory(forward02);
                changeLift(2500);
                Thread.sleep(1500);
                outtake();

                changeLift(0);
                drive.followTrajectory(park0);
                drive.followTrajectory(park02);

            }

            if (position == 2) {
                drive.followTrajectory(forward3);
                drive.followTrajectory(forward3_2);
                outtake2();

                drive.followTrajectory(forward03);
                changeLift(2500);
                Thread.sleep(3000);
                outtake();

                changeLift(0);
                drive.followTrajectory(park03);
                drive.followTrajectory(park03_2);
            }





        }

    }
}
