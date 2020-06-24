package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Bobat;

//@Autonomous(name="LoadBlue", group="Drive")
public class LoadBlue extends LinearOpMode {
    //br,bl,fr,fl
    //rightintake: 1
    //leftintake: 2
    //rotary: 0
    Bobat robot = new Bobat();
    ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.4;

    @Override

    public void runOpMode(){
        robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fServo2.setPosition(0);
        waitForStart();
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*encodeDrive(0.3, 8, -8, -8, 8); //strafe right
        encodeDrive(0.3, -12, 12, 12, -12);//strafe left
        encodeDrive(0.3, 22, -22, 22, -22); //turn right
        encodeDrive(0.3, 11, 11, 11, 11); //forward
        encodeDrive(0.3, -11, -11, -11, -11); //backwards
        encodeDrive(0.3, -11, 11, -11, 11); //turn left*/

        encodeDrive(0.3, 33, -33, -33, 33);//strafe right
        //start vuforia
        blrDriveSafety(0.3,-13);
        //encodeDrive(0.3, -20, -20, -20, -20);//drive backwards while scannning
        encodeDrive(0.3, -21, 21, -21, 21);//once skystone is found rotate left
        blrDriveSafety(0.3,-7);
        //encodeDrive(0.3, -5, -5, -5, -5);//drive backwards towards the stone
        robot.fServo2.setPosition(0.35);//drop fservo2
        sleep(1000);
        blrDriveSafety(0.3,12);
        //encodeDrive(0.3, 10, 10, 10, 10);
        encodeDrive(0.3, -22, 22, -22, 22);//rotate left
        blrDriveSafety(0.3,-70);
        //encodeDrive(0.3, -65, -65, -65, -65);//drive backwards under bridge
        robot.fServo2.setPosition(0);//lift servo
        blrDriveSafety(0.3,13);
        //encodeDrive(0.3, 8, 8, 8, 8);//drive forward


        //Values that Robot Goes (Do Opposite to make Code Right):
        // Straight(x, x + 1) when power is 0.2
        // Strafe(x, x - 2) when power is 0.3

        telemetry.addData("Path", "Complete");
    }

    public void encodeDrive(double power, double fLeft, double fRight, double bLeft, double bRight){
        int newFL;
        int newFR;
        int newBL;
        int newBR;

        if(opModeIsActive()){
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newFL = (int)(fLeft * COUNTS_PER_INCH);
            newFR = (int)(fRight * COUNTS_PER_INCH);
            newBL = (int)(bLeft * COUNTS_PER_INCH);
            newBR = (int)(bRight * COUNTS_PER_INCH);

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
    public void blrDriveSafety(double power, int distance){
        int newBL;
        int newBR;
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if(opModeIsActive()){
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newBL = (int)(distance * COUNTS_PER_INCH);
            newBR = (int)(distance * COUNTS_PER_INCH);

            robot.backLeft.setTargetPosition(newBL);
            robot.backRight.setTargetPosition(newBR);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setPower(power);
            robot.backRight.setPower(power);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 7.0 &&
                    robot.backLeft.isBusy() && robot.backRight.isBusy())
            {
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


    public void flDrive(double power, double fLeft){
        int newFL;

        if(opModeIsActive()){
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newFL = (int)(fLeft * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newFL);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setPower(power);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 7.0 &&
                    robot.frontLeft.isBusy() && newFL > Math.abs(robot.frontLeft.getCurrentPosition())) {
                telemetry.addData("FrontLeft Location:", "%7d", -robot.frontLeft.getCurrentPosition());
                telemetry.update();
            }
            robot.frontLeft.setPower(0.0);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1000);
        }
    }
    public void flrDrive(double power, double fLeft, double fRight){
        int newFL;
        int newFR;

        if(opModeIsActive()){
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newFL = (int)(fLeft * COUNTS_PER_INCH);
            newFR = (int)(fRight * COUNTS_PER_INCH);

            robot.frontLeft.setTargetPosition(newFL);
            robot.frontRight.setTargetPosition(newFR);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setPower(power);
            robot.frontRight.setPower(power);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 10.0 &&
                    robot.frontLeft.isBusy() && robot.frontRight.isBusy())
            {
                telemetry.addData("FrontLeft Location:", "%7d", -robot.frontLeft.getCurrentPosition());
                telemetry.addData("FrontRight Location:", "%7d", -robot.frontRight.getCurrentPosition());
                telemetry.update();
            }
            robot.frontLeft.setPower(0.0);
            robot.frontRight.setPower(0.0);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1000);
        }
    }

}