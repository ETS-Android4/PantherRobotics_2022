package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class DriveTest extends LinearOpMode {

    private DcMotor leftmotor;
    private DcMotor rightmotor;

    @Override
    public void runOpMode() {

        leftmotor = hardwareMap.get(DcMotor.class, "leftdrive");
        rightmotor = hardwareMap.get(DcMotor.class, "rightdrive");



        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        rightmotor.setDirection(DcMotor.Direction.FORWARD);

        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        waitForStart();


        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            leftmotor.setPower(leftPower);
            rightmotor.setPower(rightPower);

        }
    }
}