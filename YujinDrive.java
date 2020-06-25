package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name="YujinCowPig<3", group="Autonomous")
public class YujinDrive extends LinearOpMode {
    public DcMotor MotorFL, MotorBL;
    public DcMotor MotorFR, MotorBR;
    public DcMotor rotMotor, intakeMotor1, intakeMotor2;
//    public Servo fServo1;
    public Servo fServo2;
    //  public Servo armServo;
    public Servo phServo;


    @Override
    public void runOpMode() {

//
//        fServo1 = hardwareMap.servo.get("fServo1");
        fServo2 = hardwareMap.servo.get("fServo2");
        phServo = hardwareMap.servo.get("ph");
        // armServo = hardwareMap.servo.get("armServo");
        MotorFL = hardwareMap.dcMotor.get("fL");
        MotorFR = hardwareMap.dcMotor.get("fR");
        MotorBL = hardwareMap.dcMotor.get("bL");
        MotorBR = hardwareMap.dcMotor.get("bR");
        intakeMotor1 = hardwareMap.dcMotor.get("in1");
        intakeMotor2 = hardwareMap.dcMotor.get("in2");
        rotMotor = hardwareMap.dcMotor.get("rot");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        fServo2.setPosition(1.0);
        while (opModeIsActive()) {
            if (gamepad1.right_trigger != 0) {
                Strafe(-gamepad1.right_trigger);
            } else if (gamepad1.left_trigger != 0) {
                Strafe(gamepad1.left_trigger);
            } else if (gamepad1.right_stick_x != 0) {//turning

                Turn(-gamepad1.right_stick_x);
            } else {
                //wao WOW OWO UWU ~Andrew Huang
                Drive(gamepad1.left_stick_y);
                //mecanum(gamepad1.left_stick_x, gamepad1.left_stick_y );
            }

            if(gamepad2.right_stick_y > 0 )
            {
                intakeMotor1.setPower(0.6);
                intakeMotor2.setPower(-0.6);
            }
            else if(gamepad2.right_stick_y < 0 )
            {
                intakeMotor1.setPower(-0.6);
                intakeMotor2.setPower(0.6);
            }
            else
            {
                intakeMotor1.setPower(0);
                intakeMotor2.setPower(0);
            }


            if (gamepad2.dpad_up && !gamepad2.x) {
                rotMotor.setPower(0.2);
            }
            else if (gamepad2.dpad_down && !gamepad2.x) {
                rotMotor.setPower(-0.2);
            }

            else if (gamepad2.dpad_up && gamepad2.x) {
                rotMotor.setPower(0.53);
            }
            else if (gamepad2.dpad_down && gamepad2.x) {
                rotMotor.setPower(-0.53);
            }
            else{
                rotMotor.setPower(0);
            }





            //FR =0, FL = 1, BR=2, BL = 3
            //finger servos
            float lastPos = -1;
            if(gamepad2.right_bumper)
            {
                if(Math.abs(lastPos - gamepad2.right_trigger) > 0.05){
//                    fServo1.setPosition(0.45);
                    fServo2.setPosition(0.53);
                    lastPos = gamepad2.right_trigger;
                }

                telemetry.update();
                telemetry.addData("finger", gamepad2.right_trigger);
            }
            else {

                fServo2.setPosition(1);
            }
            float lastPos1 = -1;


            if(gamepad2.left_bumper)
            {
                phServo.setPosition(0.47);
                telemetry.addData("ph", gamepad2.left_trigger);
                telemetry.update();
            }

            else if(gamepad2.right_bumper) //closer
            {
                phServo.setPosition(0.22);
                telemetry.addData("ph", gamepad2.left_trigger);
                telemetry.update();
            }

            else
            {
                phServo.setPosition(0.3);
            }

            //phServo.setPosition(gamepad2.left_trigger);



        }
    }

    //Front is 223rpm, Back is 312rpm
    void Drive (float speed){
        if(gamepad1.a) {
            MotorFL.setPower(0.1*speed);
            MotorBL.setPower(0.1*speed);
            MotorFR.setPower(0.1*-speed);
            MotorBR.setPower(0.1*-speed);
        }
        if(gamepad1.b) {
            MotorFL.setPower(0.9*speed);
            MotorBL.setPower(0.9*speed);
            MotorFR.setPower(0.9*-speed);
            MotorBR.setPower(0.9*-speed);
        }
        else{
            MotorFL.setPower(0.6 * speed);
            MotorBL.setPower(0.6 * speed);
            MotorFR.setPower(0.6 * -speed);
            MotorBR.setPower(0.6 * -speed);
        }
    }

    void Turn (float speed){
        if(gamepad1.a)
        {
            MotorFL.setPower(0.3*speed);
            MotorBL.setPower(0.3*speed);
            MotorFR.setPower(0.3*speed);
            MotorBR.setPower(0.3*speed);

        }
        if(gamepad1.b)
        {
            MotorFL.setPower(0.9*speed);
            MotorBL.setPower(0.9*speed);
            MotorFR.setPower(0.9*speed);
            MotorBR.setPower(0.9*speed);

        }
        else {
            MotorFL.setPower(0.6 * speed);
            MotorBL.setPower(0.6 * speed);
            MotorFR.setPower(0.6 * speed);
            MotorBR.setPower(0.6 * speed);
        }
    }

    void Strafe (float speed){
        if(gamepad1.a)
        {
            MotorFL.setPower(0.3*speed);
            MotorBL.setPower(0.3*-speed);
            MotorFR.setPower(0.3*speed);
            MotorBR.setPower(0.3*-speed);
        }
        if(gamepad1.b)
        {
            MotorFL.setPower(0.9*speed);
            MotorBL.setPower(0.9*-speed);
            MotorFR.setPower(0.9*speed);
            MotorBR.setPower(0.9*-speed);
        }
        else {
            MotorFL.setPower(0.6 * speed);
            MotorBL.setPower(0.6 * -speed);
            MotorFR.setPower(0.6 * speed);
            MotorBR.setPower(0.6 * -speed);
        }
    }

    /**
     * Control a mecanum drive base with three double inputs
     *
     * @param Strafe  is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
    //* @param Turn    is the second double X value which represents how the base should turn
     */
    public void mecanum(double Strafe, double Forward) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        // double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = (r * Math.cos(robotAngle)); // + rightX;
        final double v2 = (r * Math.sin(robotAngle)); // - rightX;
        final double v3 = (r * Math.sin(robotAngle)); // + rightX;
        final double v4 = (r * Math.cos(robotAngle)); // - rightX;

        MotorFL.setPower(v1);
        MotorFR.setPower(v2);
        MotorBL.setPower(v3);
        MotorBR.setPower(v4);
    }

}


