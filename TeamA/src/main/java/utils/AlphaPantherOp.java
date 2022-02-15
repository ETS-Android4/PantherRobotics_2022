package utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public abstract class AlphaPantherOp extends LinearOpMode
{
    protected final ElapsedTime runtime = new ElapsedTime();

    protected DcMotor lfWheel, rfWheel, lbWheel, rbWheel;

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    double globalAngle;

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();
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

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        lfWheel.setPower(leftPower);
        lbWheel.setPower(leftPower);
        rfWheel.setPower(rightPower);
        rbWheel.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        lfWheel.setPower(0);
        lbWheel.setPower(0);
        rfWheel.setPower(0);
        rbWheel.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

}
