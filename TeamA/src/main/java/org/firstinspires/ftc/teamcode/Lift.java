package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Lift extends LinearOpMode {

    private DcMotor lift;


    @Override
    public void runOpMode() {

        lift = hardwareMap.get(DcMotor.class, "liftArm");




        lift.setDirection(DcMotor.Direction.FORWARD);


        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        waitForStart();
        final double liftSpeed=0.5;

        while (opModeIsActive()) {

            if (gamepad1.left_trigger>0) {
                lift.setPower(liftSpeed);
            }
            else if(gamepad1.right_trigger>0){
                lift.setPower(-liftSpeed);
            }
            else{
                lift.setPower(0);
            }

        }
    }
}