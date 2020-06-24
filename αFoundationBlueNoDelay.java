
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.hardware.Sensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Bobat;

import java.util.Locale;
@Autonomous(name = "IMUFoundBlueNoDelay", group = "Autonomous")
public class αFoundationBlueNoDelay extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.4;


    Bobat r = new Bobat();
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, rotPower = 0.8, rotation;
    boolean                 aButton, bButton;
    PIDController           pidRotate;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        r.init(hardwareMap);
        r.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.fServo2.setPosition(0.1);
        r.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        //0.00003
        pidRotate = new PIDController(.00667, 0.0000667, 0);
        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        //pidDrive = new PIDController(.05, 0, 0);

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

        // wait for start button.

        waitForStart();
        encodeDrive(0.3, -15, 15, 15, -15);//strafe left
        blrDriveSafety(0.3, -30);//drive backwards
        r.fServo2.setPosition(0.6);//deploy the servo
        sleep(600);
        encodeDrive(0.3, 20, 20, 20, 20);
        rotate(-90,0.6);
        r.fServo2.setPosition(0.1);
        sleep(300);
        blrDriveSafety(0.3, 5);
        blrDriveSafety(0.4, -30);
        blrDriveSafety(0.3, 10);
        pidRotate = new PIDController(0.00333, 0.0000333,0);
        rotate(180,0.6);
        encodeDrive(0.3, 17,-17,-17,17);
        while (opModeIsActive() && (runtime.seconds() < 3.5f)) {
            r.tape.setPower(1.0);
        }

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
     * @return Angle in degrees. + = left, - = right from zero point.
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
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        r.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                r.backLeft.setPower(power);
                r.frontLeft.setPower(power);
                r.backRight.setPower(-power);
                r.frontRight.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                r.backLeft.setPower(-power);
                r.frontLeft.setPower(-power);
                r.backRight.setPower(power);
                r.frontRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle());
                r.backLeft.setPower(-power);
                r.frontLeft.setPower(-power);
                r.backRight.setPower(power);
                r.frontRight.setPower(power);// power will be + on left turn.
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        r.backLeft.setPower(0.0);
        r.frontLeft.setPower(0.0);
        r.backRight.setPower(0.0);
        r.frontRight.setPower(0.0);

        rotation = getAngle();
        r.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // wait for rotation to stop.
        sleep(400);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void encodeDrive(double power, double fLeft, double fRight, double bLeft, double bRight) {
        int newFL;
        int newFR;
        int newBL;
        int newBR;

        if (opModeIsActive()) {
            r.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            newFL = (int) (fLeft * COUNTS_PER_INCH);
            newFR = (int) (fRight * COUNTS_PER_INCH);
            newBL = (int) (bLeft * COUNTS_PER_INCH);
            newBR = (int) (bRight * COUNTS_PER_INCH);

            r.frontLeft.setTargetPosition(0);
            r.frontRight.setTargetPosition(0);
            r.backLeft.setTargetPosition(0);
            r.backRight.setTargetPosition(0);


            r.frontLeft.setTargetPosition(newFL);
            r.frontRight.setTargetPosition(newFR);
            r.backLeft.setTargetPosition(newBL);
            r.backRight.setTargetPosition(newBR);

            r.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.frontLeft.setPower(power);
            r.frontRight.setPower(power);
            r.backLeft.setPower(power);
            r.backRight.setPower(power);

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 7 &&
                    r.frontLeft.isBusy() && r.frontRight.isBusy() && r.backLeft.isBusy() &&
                    r.backRight.isBusy()
            ) {
                telemetry.addData("FrontLeft Location:", "%7d", r.frontLeft.getCurrentPosition());
                telemetry.addData("FrontRight Location:", "%7d", r.frontRight.getCurrentPosition());
                telemetry.addData("BackLeft Location:", "%7d", (r.backLeft.getCurrentPosition()));
                telemetry.addData("BackRight Location:", "%7d", (r.backRight.getCurrentPosition()));
                telemetry.update();
            }
            r.frontLeft.setPower(0.0);
            r.frontRight.setPower(0.0);
            r.backLeft.setPower(0.0);
            r.backRight.setPower(0.0);

            r.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4f)) {
            }
        }
    }
    public void blrDriveSafety(double power, int distance) {
        int newBL;
        int newBR;
        r.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        r.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (opModeIsActive()) {
            r.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            r.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newBL = (int) (distance * COUNTS_PER_INCH);
            newBR = (int) (distance * COUNTS_PER_INCH);

            r.backLeft.setTargetPosition(0);
            r.backRight.setTargetPosition(0);
            r.backLeft.setTargetPosition(newBL);
            r.backRight.setTargetPosition(newBR);
            r.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.backLeft.setPower(power);
            r.backRight.setPower(power);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 7.0 &&
                    r.backLeft.isBusy() && r.backRight.isBusy()) {
                telemetry.addData("BackLeft Location:", "%7d", r.backLeft.getCurrentPosition());
                telemetry.addData("BackRight Location:", "%7d", r.backRight.getCurrentPosition());
                telemetry.update();
            }
            r.backLeft.setPower(0.0);
            r.backRight.setPower(0.0);
            r.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            r.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            r.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4f)) {

            }

        }
    }



}