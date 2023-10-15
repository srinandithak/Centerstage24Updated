package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test", group="Linear Opmode")
public class test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx liftMotor;
    //need to declare rest

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Robot is Initialized");
        telemetry.update();

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
//        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
//
//        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER)
//
//        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Lift
            if (gamepad2.left_trigger != 0 ){
                liftMotor.setPower(-0.7);
            }

        }

    }

}