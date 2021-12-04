package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TippingPoint extends LinearOpMode {

    private DcMotor lift;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo servoMotor;

    @Override
    public void runOpMode() {

        lift = hardwareMap.get(DcMotor.class, "liftArm");
        leftMotor = hardwareMap.get(DcMotor.class, "leftDrive");
        rightMotor = hardwareMap.get(DcMotor.class, "rightDrive");
        servoMotor = hardwareMap.get(Servo.class, "testServo");

        lift.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        waitForStart();
        final double liftSpeed=0.5;

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            double v=0.42;
            if (gamepad1.left_trigger>0) {
                lift.setPower(liftSpeed);
            }
            else if(gamepad1.right_trigger>0){
                lift.setPower(-liftSpeed);
            }

            else{
                lift.setPower(0);
            }

            if(gamepad1.dpad_up){
                servoMotor.setPosition(v);

            }
            else if(gamepad1.dpad_down){
                servoMotor.setPosition(0);
            }

        }
    }
}