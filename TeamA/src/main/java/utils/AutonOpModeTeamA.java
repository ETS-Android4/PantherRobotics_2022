package utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Disabled
public abstract class AutonOpModeTeamA extends LinearOpMode {
    protected DcMotor lWheel, rWheel;
    protected BNO055IMU imu;
    // State used for updating telemetry
    protected Orientation angles;
    protected Acceleration gravity;

    // Number of ticks for every full revolution of the motor shaft - specific to our Matrix 12V DcMotors
    private static final double COUNTS_PER_MOTOR_REV = 1478.4;
    // Depends on gearing ratio between motor and wheel
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    // For figuring circumference
    private static final double WHEEL_DIAMETER_MM = 78.0;
    //This is the amount of ticks we have every mm travelled by the wheel
    private static final double COUNTS_PER_MM =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * 3.1415);

    private boolean m_initialised = false;
    private boolean m_sleepPostMove = false; // Set to true for the robot to momentarily sleep after each move
    private long m_sleepAmountMs = 100; // Sleeps for 100 milliseconds after each move

    protected void initRobot()
    {
        lWheel = hardwareMap.get(DcMotor.class, "leftWheel");
        rWheel = hardwareMap.get(DcMotor.class, "rightWheel");

        rWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // setting PIDF values
        // this tunes the motors for RUN_USING_ENCODER and RUN_TO_POSITION
        PIDFmanager.setPIDF(lWheel);
        PIDFmanager.setPIDF(rWheel);

        lWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        // Set up our telemetry dashboard
        composeTelemetry();

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

        int lTarget, rTarget;

        // Calculate new encoder count targets for drive motors
        lTarget = lWheel.getCurrentPosition() + (int) (driveDistanceMm * COUNTS_PER_MM);
        rTarget = rWheel.getCurrentPosition() + (int) (driveDistanceMm * COUNTS_PER_MM);

        encoderDrive(lTarget, rTarget, Math.min(speed, 1.0));

    }

    private void encoderDrive(int newLTarget, int newRTarget, double lSpeed, double rSpeed)
    {
        // Sets new targets for the drive motors
        lWheel.setTargetPosition(newLTarget);
        rWheel.setTargetPosition(newRTarget);


        // Turn on RUN_TO_POSITION
        lWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        lWheel.setPower(lSpeed);
        rWheel.setPower(rSpeed);

        while(opModeIsActive() &&
                (lWheel.isBusy() && rWheel.isBusy()))
        {
            /*telemetry.addData("Target Path (lf, rf, lb, rb)", "Running at: %f, %f, %f, %f", lfWheel.getTargetPosition(),  rfWheel.getTargetPosition(),  lbWheel.getTargetPosition(),  rbWheel.getTargetPosition());
            telemetry.addData("Target Path (lf, rf, lb, rb)", "Running at: %f, %f, %f, %f", lfWheel.getCurrentPosition(), rfWheel.getCurrentPosition(), lbWheel.getCurrentPosition(), rbWheel.getCurrentPosition());

            telemetry.update();*/
        }

        // Stop motion
        lWheel.setPower(0);
        rWheel.setPower(0);

        // Turn off RUN_TO_POSITION
        lWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(m_sleepPostMove)
        {
            sleep(m_sleepAmountMs);
        }
    }

    private void encoderDrive(int newLTarget, int newRTarget, double speed)
    {
        encoderDrive(newLTarget, newRTarget, speed, speed);
    }


    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}