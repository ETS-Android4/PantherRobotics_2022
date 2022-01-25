package org.firstinspires.ftc.teamcode;

import utils.Toggle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TippingPoint extends LinearOpMode {

    private DcMotor lift;
    private DcMotor leftB;
    private DcMotor rightF;
    private DcMotor leftF;
    private DcMotor rightB;
    private Servo servoMotor;

    @Override
    public void runOpMode() {

        lift = hardwareMap.get(DcMotor.class, "liftArm");
        leftF = hardwareMap.get(DcMotor.class, "leftFront");
        rightF = hardwareMap.get(DcMotor.class, "rightFront");
        leftB = hardwareMap.get(DcMotor.class, "leftBack");
        rightB = hardwareMap.get(DcMotor.class, "rightBack");
        servoMotor = hardwareMap.get(Servo.class, "testServo");

        lift.setDirection(DcMotor.Direction.FORWARD);
        leftF.setDirection(DcMotor.Direction.REVERSE);
        rightF.setDirection(DcMotor.Direction.FORWARD);
        rightB.setDirection(DcMotor.Direction.FORWARD);
        leftB.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        waitForStart();
        final double liftSpeed=1;
        double driveMult = 0.5;
        Toggle tog = new Toggle();
        tog.update(false);
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;


            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0) * driveMult;
            rightPower = Range.clip(drive - turn, -1.0, 1.0) * driveMult;

            leftF.setPower(leftPower);
            leftB.setPower(leftPower);
            rightB.setPower(rightPower);
            rightF.setPower(rightPower);


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

            tog.update(this.gamepad1.a);

            if(tog.getState()) {
                driveMult = 1;
            }
            else {
                driveMult = 0.5;
            }
        }
    }
}