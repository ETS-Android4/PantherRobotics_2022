package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import utils.AlphaPantherOp;

public class testAuton extends AlphaPantherOp {
    @Override
    public void runOpMode() {
        initRobot();
        tankDrive(500, 0.5);

    }
}