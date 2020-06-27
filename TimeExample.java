package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

//@Autonomous(name = "LoadingBlue", group = "Autonomous")
public class TimeExample extends LinearOpMode {
    // ColorSensor sensorColor;

    public DcMotor MotorFL, MotorBL;
    public DcMotor MotorFR, MotorBR;
    public Servo fServo2;
    public Servo armServo;
    public Servo phServo;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // sensorColor = hardwareMap.get(ColorSensor.class, "color");

        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        //servo = hardwareMap.servo.get("test_servo");
//        fServo1 = hardwareMap.servo.get("fServo1");
        fServo2 = hardwareMap.servo.get("fServo2");
        phServo = hardwareMap.servo.get("ph");
        armServo = hardwareMap.servo.get("armServo");
        MotorFL = hardwareMap.dcMotor.get("flmotor");
        MotorFR = hardwareMap.dcMotor.get("frmotor");
        MotorBL = hardwareMap.dcMotor.get("blmotor");
        MotorBR = hardwareMap.dcMotor.get("brmotor");
        // rotMotor = hardwareMap.dcMotor.get("rot");
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());

        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        fServo2.setPosition(0);
        fServo2.setPosition(0);
        waitForStart();


// add this before the while loop

        boolean senseblue = false;
        boolean sensered = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        waitForStart();

        Drive(0.35f);
        while (opModeIsActive() && (runtime.seconds() < 2.0f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
        }

        Drive(0);
        while (opModeIsActive() && (runtime.seconds() < 3.5f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 seconds
        }

        Drive(0);
        while (opModeIsActive() && (runtime.seconds() < 3.9f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //deploy fingers (servo code)
        }

        fServo2.setDirection(Servo.Direction.REVERSE);
        fServo2.setPosition(0.5);
        fServo2.setPosition(0.5);
        while (opModeIsActive() && (runtime.seconds() < 4.7f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 second
        }
        Drive(-0.35f); //back
        while (opModeIsActive() && (runtime.seconds() < 6.1f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
        }

        Drive(0);

        while (opModeIsActive() && (runtime.seconds() < 8.2f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 second
        }

        Turn(-0.35f); //direction

        while (opModeIsActive() && (runtime.seconds() < 8.7f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 second
        }

        Turn(0f);

        while (opModeIsActive() && (runtime.seconds() < 11.0f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 second
        }

        Drive(0.35f);

        while (opModeIsActive() && (runtime.seconds() < 13.0f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 second
        }

        Drive(0);

        while (opModeIsActive() && (runtime.seconds() < 15.0f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 second
        }



        Drive(0);

        while (opModeIsActive() && (runtime.seconds() < 15.0f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 second
        }

        fServo2.setPosition(0);
        fServo2.setPosition(1);

        while (opModeIsActive() && (runtime.seconds() < 17.0f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 second
        }

        Drive(-1);

        while (opModeIsActive() && (runtime.seconds() < 15.0f)) {
            telemetry.addData("Status", "Wait");
            telemetry.update();
            //wait for 0.2 second
        }



    }


    void Drive ( float speed){
        MotorFL.setPower(speed);
        MotorBL.setPower(speed);
        MotorFR.setPower(-speed);
        MotorBR.setPower(-speed);
    }

    void Turn ( float speed){
        MotorFL.setPower(speed);
        MotorBL.setPower(speed);
        MotorFR.setPower(speed);
        MotorBR.setPower(speed);
    }

    void Strafe ( float speed){
        MotorFL.setPower(speed);
        MotorBL.setPower(-speed);
        MotorFR.setPower(speed);
        MotorBR.setPower(-speed);
    }


}

