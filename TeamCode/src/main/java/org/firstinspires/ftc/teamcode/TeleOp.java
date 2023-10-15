package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Hardware
    public DcMotorEx liftMotor;
    public DcMotorEx intakeMotor;
    public CRServo outtake;


    //otherVariables

    public boolean turtleMode = false;
    public static final double NORMAL_SPEED = 0.75;
    public static final double TURTLE_SPEED = 0.25;
    public double robotSpeed = NORMAL_SPEED;
    public double pickPosition = .65;
    public double dropPosition = .37;
    public double rotationSpeed = .75;
    public boolean fieldOriented = false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Robot is Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        outtake = hardwareMap.crservo.get("outtake");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            //clawMotor
            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                rightClaw.setPosition(pickPosition);
                leftClaw.setPosition(0);
            } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
                rightClaw.setPosition(dropPosition);
                leftClaw.setPosition(.3);
            }

            //Lift
            if (gamepad2.left_trigger != 0 || gamepad1.left_trigger != 0 && liftMotor.getCurrentPosition() >= 0) {
                liftMotor.setPower(-0.9);
            } else if (gamepad2.right_trigger != 0 || gamepad1.right_trigger != 0 && liftMotor.getCurrentPosition() <= 4200) {
                liftMotor.setPower(0.85);
            } else {
                liftMotor.setPower(0.0);
                if ((gamepad2.left_trigger == 0 && gamepad1.left_trigger == 0) && (!gamepad1.b && !gamepad2.b) && liftMotor.getCurrentPosition() >= 100) {
                    liftMotor.setPower(0.02);
                } else {
                    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }

//            //lift reset
            if ((gamepad2.b || gamepad1.b) && liftMotor.getCurrentPosition() > 0) {
                liftMotor.setPower(-0.50);
            }

            //intake
            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                intakeMotor.setPower(0.9);
            } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
                intakeMotor.setPower(-0.9);
            } else {
                intakeMotor.setPower(0.0);
            }

            //outtake
            while (gamepad1.y || gamepad2.y) {
                outtake.setPower(1);
            }

            //turtleMode
            if (gamepad1.dpad_up && !turtleMode) {
                turtleMode = true;
                robotSpeed = TURTLE_SPEED;
            } else if (gamepad1.dpad_down && turtleMode) {
                turtleMode = false;
                robotSpeed = NORMAL_SPEED;
            }

            //fieldOriented toggle
            if (gamepad1.a && !fieldOriented) {
                drive.setPoseEstimate(new Pose2d(0,0,0));
                fieldOriented = true;
            } else if (gamepad1.x && fieldOriented) {
                fieldOriented = false;
            }


            //movement
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            if (fieldOriented) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX() * robotSpeed,
                                input.getY() * robotSpeed,
                                -gamepad1.right_stick_x * robotSpeed * rotationSpeed
                        )
                );
            }

            else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * robotSpeed,
                                -gamepad1.left_stick_x * robotSpeed,
                                -gamepad1.right_stick_x * robotSpeed * rotationSpeed
                        )
                );
            }

            // Update everything. Odometry. Etc.
            drive.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("DRIVE", "------------------------------------");
            telemetry.addData("DriveMode: ", (turtleMode) ? ("turtleMode") : ("Normal"));
            telemetry.addData("OTHER", "------------------------------------");
            telemetry.addData("LiftMotor Position: ", liftMotor.getCurrentPosition());
            telemetry.addData("DriveType: ", (fieldOriented) ? ("Field-Oriented Drive") : ("Robot-Oriented"));

        }
    }
}





