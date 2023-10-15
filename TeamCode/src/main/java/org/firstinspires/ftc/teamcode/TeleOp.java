+package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx liftMotor;
    public DcMotorEx liftMotor2;
    //need to declare rest

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Robot is Initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");

        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER)

        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Lift
            if (gamepad2.left_trigger != 0 || gamepad1.left_trigger != 0 && liftMotor.getCurrentPosition() >= 0){
                liftMotor.setPower(-0.7);
                liftMotor2.setPower(0.7);
            } else if (gamepad2.right_trigger != 0 || gamepad1.right_trigger != 0 && liftMotor.getCurrentPosition() <= 4200) {
                if (liftMotor.getCurrentPosition() >= 1500) {
                    liftMotor.setPower(0.3);
                    liftMotor2.setPower(-0.3);
                }
                liftMotor.setPower(0.85);
                liftMotor2.setPower(-0.85);
            } else {
                liftMotor.setPower(0.0);
                liftMotor2.setPower(0.0);
                if ((gamepad2.left_trigger == 0 && gamepad1.left_trigger == 0) && (!gamepad1.b && !gamepad2.b) && liftMotor.getCurrentPosition() >= 1000) {
                    liftMotor.setPower(0.01);
                    liftMotor2.setPower(-0.01);
                } else {
                    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
        }

    }

}