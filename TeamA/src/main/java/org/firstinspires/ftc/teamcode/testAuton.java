package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import utils.AlphaPantherOp;
@Autonomous
public class testAuton extends AlphaPantherOp {
    @Override

    public void runOpMode() {
        initRobot();

        int newPos = lift.getCurrentPosition() - 1456;


        lift.setTargetPosition(newPos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.3);
        telemetry.addData("Moving lift", 50);
        telemetry.update();
        while(lift.isBusy()) {}
        lift.setPower(0);
        tankDrive(500, 0.5, false);

    }
}