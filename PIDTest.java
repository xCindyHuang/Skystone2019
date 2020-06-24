package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
//@Autonomous(name="PIDTest", group="SkystoneAuto")
public class PIDTest extends LinearOpMode{
    //br,bl,fr,fl
    //rightintake: 1
    //leftintake: 2
    //rotary: 0
    Bobat robot = new Bobat();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.4;

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

        waitForStart();
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        testMaxVelocity();
    }

    public void blrDriveSafety(double power, int distance){
        int newBL;
        int newBR;
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if(opModeIsActive()) {
            PIDFCoefficients newPIDF = new PIDFCoefficients(5.0, 0.0, 0.0, 0.0);
//            robot.backLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
//            robot.backRight.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            while (opModeIsActive() && runtime.seconds() < 8.0 &&
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
    public void testMaxVelocity(){
        double maxVel = 0;
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeft.setPower(1.0);
        robot.backRight.setPower(1.0);
        runtime.reset();
        while(runtime.seconds() < 10.0){
//            if(robot.backLeft.getVelocity() > maxVel)
//                maxVel = robot.backLeft.getVelocity();
//            telemetry.addData("Current Velocity: %7d", robot.backLeft.getVelocity());
//            telemetry.addData("Max Velocity: %7d", maxVel);
//        }
        robot.backLeft.setPower(0.0);
        robot.backRight.setPower(0.0);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}}