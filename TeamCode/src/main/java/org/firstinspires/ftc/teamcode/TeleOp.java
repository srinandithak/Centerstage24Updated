package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Hardware: Declaring all the robot parts
    public DcMotorEx liftMotor;
    public DcMotorEx intakeMotor;

    public DcMotorEx rightSuspension;
    public DcMotorEx leftSuspension;

    public CRServo intakeServo;
    public CRServo outtake;

    public Servo stackServo;

    public Servo droneLauncher;

    public Servo tiltServo;
    public Servo rightSuspensionServo;
    public Servo leftSuspensionServo;

    public Servo rightRampServo;
    public Servo leftRampServo;

    public DistanceSensor distanceSensorLeft;

    public DistanceSensor distanceSensorRight;

    public double initialPositionRight;
    public double initialPositionLeft;


    //Reduces speed when true
    public boolean turtleMode = false;

    //Other variables
    public static final double NORMAL_SPEED = 1;
    public static final double TURTLE_SPEED = 0.4;
    public double robotSpeed = NORMAL_SPEED;
    public double rotationSpeed = 1;
    public boolean fieldOriented = false;

    public double idealPosition() {
        return 0;
    }


    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Status", "Robot is Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Most robots need the motor on one side to be reversed to drive forward - was done in Sample Mecanum Drive
        // Reverse the motor that runs backwards when connected directly to the battery
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        rightSuspension = hardwareMap.get(DcMotorEx.class, "rightSuspension");
        leftSuspension = hardwareMap.get(DcMotorEx.class, "leftSuspension");
        intakeServo = hardwareMap.crservo.get("intakeServo");
        leftSuspensionServo = hardwareMap.get(Servo.class, "leftSuspensionServo");
        rightSuspensionServo = hardwareMap.get(Servo.class, "rightSuspensionServo");
        stackServo = hardwareMap.get(Servo.class, "stackServo");
        outtake = hardwareMap.crservo.get("outtake");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        rightRampServo = hardwareMap.get(Servo.class, "rightRampServo");
        leftRampServo = hardwareMap.get(Servo.class, "leftRampServo");
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");


        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
        initialPositionLeft = leftRampServo.getPosition();
        initialPositionRight = rightRampServo.getPosition();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // Vrishank (the goat) was here

        while (opModeIsActive()) {




//            double d1 = distanceSensorLeft.getDistance(DistanceUnit.MM);
//
//            double d2 = distanceSensorRight.getDistance(DistanceUnit.MM);

            // autoposition

            rightRampServo.setDirection(rightRampServo.getDirection().REVERSE);
            // ramp servo positions
            telemetry.addData("Right Ramp Servo Position", rightRampServo.getPosition());
            telemetry.addData("Left Ramp Servo Position", leftRampServo.getPosition());
//            telemetry.addData("Left Distance Sensor", distanceSensorLeft.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Right Distance Sensor", distanceSensorRight.getDistance(DistanceUnit.INCH));

            telemetry.update();



            //distance sensor
//            if (gamepad1.dpad_left) {
//
//                if (distanceSensorLeft.getDistance(DistanceUnit.INCH) >= distanceSensorRight.getDistance(DistanceUnit.INCH)) {
//                    while(distanceSensorLeft.getDistance(DistanceUnit.INCH) >= distanceSensorRight.getDistance(DistanceUnit.INCH)) {
//                        if (gamepad1.dpad_right) break;
//                        drive.setWeightedDrivePower(
//                                new Pose2d(
//                                        0,
//                                        0,
//                                        -0.2
//                                )
//                        );
//                    }
//                }
//
//                else {
//                    while(distanceSensorRight.getDistance(DistanceUnit.INCH) >= distanceSensorLeft.getDistance(DistanceUnit.INCH)) {
//                        if (gamepad1.dpad_right) break;
//                        drive.setWeightedDrivePower(
//                                new Pose2d(
//                                        0,
//                                        0,
//                                        0.2
//                                )
//                        );
//                    }
//                }
//
//                if (distanceSensorLeft.getDistance(DistanceUnit.INCH) > 8.5) {
//                    while (distanceSensorLeft.getDistance(DistanceUnit.INCH) >= 8.5) {
//                        drive.setWeightedDrivePower(
//                                new Pose2d(
//                                        0.2,
//                                        0,
//                                        0
//                                )
//                        );
//                    }
//                }
//            }



//
//                } else if (distanceSensorLeft.getDistance(DistanceUnit.INCH) > 7.559) {
//                    while (distanceSensorLeft.getDistance(DistanceUnit.INCH) >= 7.559) {
//                        drive.setWeightedDrivePower(
//                                new Pose2d(
//                                        -gamepad1.left_stick_y * robotSpeed,
//                                        -gamepad1.left_stick_x * robotSpeed,
//                                        -gamepad1.right_stick_x * robotSpeed * rotationSpeed
//                                )
//                        );
//                    }
//                }

            //droneLauncher
            if (gamepad1.x || gamepad2.x) {
                tiltServo.setPosition(0.25);
                droneLauncher.setPosition(0);
                Thread.sleep(1500);
                tiltServo.setPosition(0);

//                droneLauncher.setPosition(0.5);

//                droneLauncher.setPosition(0);
            }

            //stackServo
            if (gamepad1.dpad_up){
                stackServo.setPosition(1);
            } else if (gamepad1.dpad_down){
                stackServo.setPosition(.2);

            }

            if (gamepad1.dpad_right) {
                liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            //rampServo
            if (liftMotor.getCurrentPosition() >= 1500) {
                //test position
                leftRampServo.setPosition(.75); //Outtake position
                robotSpeed = TURTLE_SPEED;

            }

            if (liftMotor.getCurrentPosition() < 1500) {

                leftRampServo.setPosition(.29); //Tilt position to avoid scraping
                robotSpeed = NORMAL_SPEED;

            }

            //intake
            if (gamepad1.right_bumper || gamepad2.right_bumper) { //reverse intake
                //robotSpeed = TURTLE_SPEED;
                intakeMotor.setPower(1);
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) { //intake
                //robotSpeed = TURTLE_SPEED;
                intakeMotor.setPower(-1);
                leftRampServo.setPosition(.25); //Intake position
            }
            else {
                //robotSpeed = NORMAL_SPEED;
                intakeMotor.setPower(0);
            }
            //middle servo

            if (gamepad1.right_bumper || gamepad2.right_bumper || gamepad1.a || gamepad2.a) {
                intakeServo.setPower(-1);
            } else if (gamepad1.left_bumper || gamepad2.left_bumper || gamepad1.y || gamepad2.y) {
                intakeServo.setPower(1);
            } else {
                intakeServo.setPower(0);
            }

            //outtake

            if (gamepad1.y || gamepad2.y){
                outtake.setPower(-1);
            } else if (gamepad1.a || gamepad2.a) {
                outtake.setPower(1);
            } else {
                outtake.setPower(0);
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




            //turtleMode
            if (gamepad1.dpad_up && !turtleMode) {
                turtleMode = true;
                robotSpeed = TURTLE_SPEED;
            } else if (gamepad1.dpad_down && turtleMode) {
                turtleMode = false;
                robotSpeed = NORMAL_SPEED;
            }

            //fieldOriented toggle
//            if (gamepad1.dpad_left && !fieldOriented) {
//                drive.setPoseEstimate(new Pose2d(0,0,0));
//                fieldOriented = true;
//            } else if (gamepad1.dpad_right && fieldOriented) {
//                fieldOriented = false;
//            }


//            if (gamepad1.dpad_down || gamepad2.dpad_down) {
//                leftSuspension.setPower(0.2);
//                rightSuspension.setPower(-0.2);
//            }
//            else if (gamepad1.dpad_up || gamepad2.dpad_up) {
//                leftSuspension.setPower(-0.2);
//                rightSuspension.setPower(0.2);
//            }
//            else {
//                leftSuspension.setPower(0);
//                rightSuspension.setPower(0);
//            }

            if (gamepad2.dpad_down) {
                leftSuspensionServo.setPosition(.45);
                rightSuspensionServo.setPosition(.45);
            } else if (gamepad2.dpad_up){
                leftSuspensionServo.setPosition(.825);
                rightSuspensionServo.setPosition(.15);
            }


//            else {
//                leftSuspensionServo.setPower(0);
//                rightSuspensionServo.setPower(0);
//            }

            if (gamepad2.dpad_right) {
                rightSuspension.setPower(0.95);
                leftSuspension.setPower(0.95);
            } else if (gamepad2.dpad_left){
                rightSuspension.setPower(-0.95);
                leftSuspension.setPower(-0.95);
            }
            else {
                rightSuspension.setPower(0);
                leftSuspension.setPower(0);
            }

            //movement
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading

           // if (d1 < 10 && d2 < 10 && d1 > 5 && d2 > 5) {
//            //    while (d1 < d2) {
////                    moveLeftWheelForward();
//                }
//            //    while (d2 < d1) {
////                    moveRightWheelForward();
//                }
//            }
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
//            if (fieldOriented) {
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                input.getX() * robotSpeed,
//                                input.getY() * robotSpeed,
//                                -gamepad1.right_stick_x * robotSpeed * rotationSpeed
//                        )
//                );
//            }

            //else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * robotSpeed,
                                -gamepad1.left_stick_x * robotSpeed,
                                -gamepad1.right_stick_x * robotSpeed * rotationSpeed
                        )
                );
            //}

            // Update everything. Odometry. Etc.
            drive.update();

            // Show the elapsed game time and wheel power.
//            telemetry.addData("tilt servo position", tiltServo.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("DRIVE", "------------------------------------");
            telemetry.addData("DriveMode: ", (turtleMode) ? ("turtleMode") : ("Normal"));
            telemetry.addData("OTHER", "------------------------------------");
            telemetry.addData("LiftMotor Position: ", liftMotor.getCurrentPosition());
            telemetry.addData("DriveType: ", (fieldOriented) ? ("Field-Oriented Drive") : ("Robot-Oriented"));
// ANKARA MESSI
            // ANKARA MESSI
            // ANKARA MESSI
            // ANKARA MESSI
            // ANKARA MESSI
            // ANKARA MESSI
            // ANKARA MESSI
            // ANKARA MESSI
            // ANKARA MESSI
            //GOL
            //GOL//GOL//GOL//GOL//GOL//GOL//GOL//GOL//GOL








        }
    }
}





