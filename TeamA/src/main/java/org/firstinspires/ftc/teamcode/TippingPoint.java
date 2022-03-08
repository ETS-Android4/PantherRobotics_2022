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

        double liftInitPos = lift.getCurrentPosition();

        waitForStart();
        final double liftSpeed=1;
        double driveMult = 0.5;
        Toggle slowToggle = new Toggle();
        // set default speed to slow
        slowToggle.update(true);

        Toggle liftLimitToggle = new Toggle();
        liftLimitToggle.update(true);

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;


            double drive = gamepad1.left_stick_y;
            double turn = (gamepad1.right_stick_x == 0) ? -gamepad1.left_stick_x
                                                        : -gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0) * driveMult;
            rightPower = Range.clip(drive - turn, -1.0, 1.0) * driveMult;

            leftF.setPower(leftPower);
            leftB.setPower(leftPower);
            rightB.setPower(rightPower);
            rightF.setPower(rightPower);


            double v=0.42;

            // lift down
            if (gamepad1.left_trigger>0 && lift.getCurrentPosition()<liftInitPos-1000) {
                lift.setPower(liftSpeed);
            }
            // lift up
            else if(gamepad1.right_trigger>0 && lift.getCurrentPosition()>liftInitPos-15800){

            // lift downwards movement is positive
            if (gamepad1.left_trigger>0 && (lift.getCurrentPosition()<liftInitPos-750 || !liftLimitToggle.getState())) {
                lift.setPower(liftSpeed);
            }
            // lift upwards movement is negative, downwards is positive
            else if(gamepad1.right_trigger>0 && (lift.getCurrentPosition()>liftInitPos-15800 || !liftLimitToggle.getState())){

                lift.setPower(-liftSpeed);
            }
            else{
                lift.setPower(0);
            }

            if(gamepad1.left_bumper){
                servoMotor.setPosition(v);
            }
            else if(gamepad1.right_bumper){
                servoMotor.setPosition(0);
            }

            slowToggle.update(this.gamepad1.a);
            liftLimitToggle.update(this.gamepad1.x);

            if(slowToggle.getState()) {
                driveMult = 0.5;
            }
            else {
                driveMult = 0.1;
            }

            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.addData("Slow Mode (A):", slowToggle.getState() ? "ON" : "OFF");
            telemetry.addData("Lift Limit (X):", liftLimitToggle.getState() ? "ON" : "OFF");
            telemetry.update();
        }
    }
}