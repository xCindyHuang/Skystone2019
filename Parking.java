package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Bobat;

@Autonomous(name = "Parking1",group = "Autonomous")
public class Parking extends LinearOpMode {
    Bobat robot = new Bobat();
    ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.4;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        waitForStart();

        while (opModeIsActive() && (runtime.seconds() < 1.5f)) {
            robot.tape.setPower(1.0);
        }

    }

    public void encodeDrive(double power, double fLeft, double fRight, double bLeft, double bRight) {
        int newFL;
        int newFR;
        int newBL;
        int newBR;

        if (opModeIsActive()) {
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newFL = (int) (fLeft * COUNTS_PER_INCH);
            newFR = (int) (fRight * COUNTS_PER_INCH);
            newBL = (int) (bLeft * COUNTS_PER_INCH);
            newBR = (int) (bRight * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newFL);
            robot.frontRight.setTargetPosition(newFR);
            robot.backLeft.setTargetPosition(newBL);
            robot.backRight.setTargetPosition(newBR);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setPower(power);
            robot.frontRight.setPower(power);
            robot.backLeft.setPower(power);
            robot.backRight.setPower(power);

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 7 &&
                    robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() &&
                    robot.backRight.isBusy()
            ) {
                telemetry.addData("FrontLeft Location:", "%7d", robot.frontLeft.getCurrentPosition());
                telemetry.addData("FrontRight Location:", "%7d", robot.frontRight.getCurrentPosition());
                telemetry.addData("BackLeft Location:", "%7d", (robot.backLeft.getCurrentPosition()));
                telemetry.addData("BackRight Location:", "%7d", (robot.backRight.getCurrentPosition()));
                telemetry.update();
            }
            robot.frontLeft.setPower(0.0);
            robot.frontRight.setPower(0.0);
            robot.backLeft.setPower(0.0);
            robot.backRight.setPower(0.0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1500);
        }
    }

    public void blrDriveSafety(double power, int distance) {
        int newBL;
        int newBR;
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (opModeIsActive()) {
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newBL = (int) (distance * COUNTS_PER_INCH);
            newBR = (int) (distance * COUNTS_PER_INCH);

            robot.backLeft.setTargetPosition(newBL);
            robot.backRight.setTargetPosition(newBR);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setPower(power);
            robot.backRight.setPower(power);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 7.0 &&
                    robot.backLeft.isBusy() && robot.backRight.isBusy()) {
                telemetry.addData("BackLeft Location:", "%7d", robot.backLeft.getCurrentPosition());
                telemetry.addData("BackRight Location:", "%7d", robot.backRight.getCurrentPosition());
                telemetry.update();
            }
            robot.backLeft.setPower(0.0);
            robot.backRight.setPower(0.0);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sleep(1500);
        }


    }
}