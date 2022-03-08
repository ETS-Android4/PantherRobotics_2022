package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import utils.AlphaPantherOp;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Auton extends AlphaPantherOp {
    @Override

    public void runOpMode() {
        initRobot(false);

        int newPos = lift.getCurrentPosition() - 1456;


        /*

        double distance = 500; //mm
        int lfTarget, rfTarget, lbTarget, rbTarget;

        // Calculate new encoder count targets for drive motors
        lfTarget = lfWheel.getCurrentPosition() + (int) (distance * COUNTS_PER_MM);
        rfTarget = rfWheel.getCurrentPosition() + (int) (distance * COUNTS_PER_MM);
        lbTarget = lbWheel.getCurrentPosition() + (int) (distance * COUNTS_PER_MM);
        rbTarget = rbWheel.getCurrentPosition() + (int) (distance * COUNTS_PER_MM);


        lfWheel.setTargetPosition(lfTarget);
        lbWheel.setTargetPosition(lbTarget);
        rfWheel.setTargetPosition(rfTarget);
        rbWheel.setTargetPosition(rbTarget);

        lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lfWheel.setPower(0.5);
        lbWheel.setPower(0.5);
        rfWheel.setPower(0.5);
        rbWheel.setPower(0.5);

        while(opModeIsActive() &&
                (lfWheel.isBusy() && rfWheel.isBusy() && lbWheel.isBusy() && rbWheel.isBusy()))
        {

        }

        lfWheel.setPower(0);
        lbWheel.setPower(0);
        rfWheel.setPower(0);
        rbWheel.setPower(0);

        */
        lift.setTargetPosition(newPos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.3);
        telemetry.addData("Moving lift", 50);
        telemetry.update();
        double timeNow = getRuntime();
        while(lift.isBusy() && getRuntime()-timeNow < 1.5) {

        }

        telemetry.addData("Thing 1", 0);
        telemetry.update();
        lift.setPower(0);


        telemetry.addData("Thing 2", 0);
        telemetry.update();
        tankDrive(500, 0.5, false);



    }
}