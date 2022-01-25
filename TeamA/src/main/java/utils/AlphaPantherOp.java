package utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
public abstract class AlphaPantherOp extends LinearOpMode
{
    protected final ElapsedTime runtime = new ElapsedTime();

    protected DcMotor lfWheel, rfWheel, lbWheel, rbWheel;

    /**

     Top down view of robot
     -----------------
     | LF         RF |
     |    *     *    |
     |       *       |
     |    *     *    |
     | LB         RB |
     -----------------

     */

    private static final double COUNTS_PER_MOTOR_REV = 1478.4;    // Number of ticks for every full revolution/rotation of the motor shaft - this is specific to our model of motors
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // Depends on gearing ratio between motor and wheel
    private static final double WHEEL_DIAMETER_MM = 78.0;     // For figuring circumference
    private static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);  //This is the amount of ticks we have every mm travelled by the wheel


    // I'm marking my private non-constant members with the 'm_' prefix.
    // Please stick to this convention, tho you likely won't have to use many private members.

    private boolean m_initialised = false;
    private boolean m_sleepPostMove = false; // Set to true for the robot to momentarily sleep after each move
    private long m_sleepAmountMs = 100; // Sleeps for 100 milliseconds after each move



    protected void initRobot()
    {
        // Mapping DcMotor objects to our real life (and gorgeously expensive) motors.
        lfWheel = hardwareMap.get(DcMotor.class, "lf");
        rfWheel = hardwareMap.get(DcMotor.class, "rf");
        lbWheel = hardwareMap.get(DcMotor.class, "lb");
        rbWheel = hardwareMap.get(DcMotor.class, "rb");

        lfWheel.setDirection(DcMotor.Direction.REVERSE);
        lbWheel.setDirection(DcMotor.Direction.REVERSE);


        // Setting the PIDF values.
        // As far as you need to know, this tunes them for RUN_USING_ENCODERS
        PIDFmanager.setPIDF(lfWheel);
        PIDFmanager.setPIDF(rfWheel);
        PIDFmanager.setPIDF(lbWheel);
        PIDFmanager.setPIDF(rbWheel);

        lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m_initialised = true;
    }

    protected void setSleepPostMove(boolean flagTrueForYes)
    {
        m_sleepPostMove = flagTrueForYes;
    }

    protected void setSleepAmount(long timeMs)
    {
        if(timeMs < 0) { return; }

        m_sleepAmountMs = timeMs;
    }



    /**
     * Method for driving robot forwards or backwards
     *
     * @param driveDistanceMm Distance you want to drive forwards, in millimetres. Negative value with perform reverse drive
     * @param speed Speed as a percentage of maximum motor speed. Range: 0 < speed <= 1
     */
    protected void tankDrive(double driveDistanceMm, double speed)
    {
        if(speed <= 0) { return; }
        if(!opModeIsActive()) { return; }

        int lfTarget, rfTarget, lbTarget, rbTarget;

        // Calculate new encoder count targets for drive motors
        lfTarget = lfWheel.getCurrentPosition() + (int) (driveDistanceMm * COUNTS_PER_MM);
        rfTarget = rfWheel.getCurrentPosition() + (int) (driveDistanceMm * COUNTS_PER_MM);
        lbTarget = lbWheel.getCurrentPosition() + (int) (driveDistanceMm * COUNTS_PER_MM);
        rbTarget = rbWheel.getCurrentPosition() + (int) (driveDistanceMm * COUNTS_PER_MM);

        encoderDrive(lfTarget, rfTarget, lbTarget, rbTarget, Math.min(speed, 1.0));

    }


    /**
     * Method for driving robot forwards or backwards
     *
     * @param driveDistanceMm Distance you want to drive forwards, in millimetres. Negative value with perform reverse drive
     * @param speed Speed as a percentage of maximum motor speed. Range: 0 < speed <= 1
     * @param reverseDrive Set to true to force reverse drive (backwards drive)
     */
    protected void tankDrive(double driveDistanceMm, double speed, boolean reverseDrive)
    {
        double negDriveDistance = - Math.abs(driveDistanceMm); // Forces distance into a negative
        tankDrive(negDriveDistance, speed);
    }

    /**
     * Method for driving robot left and right
     *
     * @param driveDistanceMm Distance you want drive right, in millimetres. Negative values will drive leftwards
     * @param speed Speed as a percentage of maximum motor speed. Range: 0 < speed <= 1
     */
    protected void strafeDrive(double driveDistanceMm, double speed)
    {
        if(speed <= 0) { return; }
        if(!opModeIsActive()) { return; }

        int lfTarget, rfTarget, lbTarget, rbTarget;

        lfTarget = lfWheel.getCurrentPosition() + (int) (driveDistanceMm * COUNTS_PER_MM);
        rfTarget = rfWheel.getCurrentPosition() - (int) (driveDistanceMm * COUNTS_PER_MM);
        lbTarget = lbWheel.getCurrentPosition() - (int) (driveDistanceMm * COUNTS_PER_MM);
        rbTarget = rbWheel.getCurrentPosition() + (int) (driveDistanceMm * COUNTS_PER_MM);

        encoderDrive(lfTarget, rfTarget, lbTarget, rbTarget, speed);
    }

    /**
     * Method for driving the robot along any linear path
     *
     * @param xDistanceMm Distance to be moved in X, in millimetres
     * @param yDistanceMm Distance to be moved in Y, in millimetres
     * @param speed Speed as a percentage of maximum motor speed. Range: 0 < speed <= 1
     */




    private void encoderDrive(int newLfTarget, int newRfTarget, int newLbTarget, int newRbTarget, double lfSpeed, double rfSpeed, double lbSpeed, double rbSpeed)
    {
        // Sets new targets for the drive motors
        lfWheel.setTargetPosition(newLfTarget);
        rfWheel.setTargetPosition(newRfTarget);
        lbWheel.setTargetPosition(newLbTarget);
        rbWheel.setTargetPosition(newRbTarget);

        // Turn on RUN_TO_POSITION
        lfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        lfWheel.setPower(lfSpeed);
        rfWheel.setPower(rfSpeed);
        lbWheel.setPower(lbSpeed);
        rbWheel.setPower(rbSpeed);

        while(opModeIsActive() &&
                (lfWheel.isBusy() && rfWheel.isBusy() && lbWheel.isBusy() && rbWheel.isBusy()))
        {
            /*telemetry.addData("Target Path (lf, rf, lb, rb)", "Running at: %f, %f, %f, %f", lfWheel.getTargetPosition(),  rfWheel.getTargetPosition(),  lbWheel.getTargetPosition(),  rbWheel.getTargetPosition());
            telemetry.addData("Target Path (lf, rf, lb, rb)", "Running at: %f, %f, %f, %f", lfWheel.getCurrentPosition(), rfWheel.getCurrentPosition(), lbWheel.getCurrentPosition(), rbWheel.getCurrentPosition());

            telemetry.update();*/
        }

        // Stop motion
        lfWheel.setPower(0);
        rfWheel.setPower(0);
        lbWheel.setPower(0);
        rbWheel.setPower(0);

        // Turn off RUN_TO_POSITION
        lfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(m_sleepPostMove)
        {
            sleep(m_sleepAmountMs);
        }
    }

    private void encoderDrive(int newLfTarget, int newRfTarget, int newLbTarget, int newRbTarget, double speed)
    {
        encoderDrive(newLfTarget, newRfTarget, newLbTarget, newRbTarget, speed, speed, speed, speed);
    }

    private void checkOrientation() {
// read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
// and save the heading
        curHeading = angles.firstAngle;
    }

    protected double timeLeft()
    {
        return 30. - runtime.time();
    }


}
