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
//akshat-was-here
//@TeleOp(name="Color", group="Autonomous")
public class ColourSensor extends LinearOpMode {
    ColorSensor sensorColor;
    //DistanceSensor sensorDistance;
    Servo servo;
    public DcMotor MotorFL, MotorBL;
    public DcMotor MotorFR, MotorBR;

    @Override
    public void runOpMode()
    {
        sensorColor = hardwareMap.get(ColorSensor.class, "color");

        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        //servo = hardwareMap.servo.get("test_servo");
        MotorFL = hardwareMap.dcMotor.get("MotorLeft");
        MotorFR = hardwareMap.dcMotor.get("MotorRight");

        //MotorBL = hardwareMap.dcMotor.get("blmotor");
        //MotorBR = hardwareMap.dcMotor.get("brmotor");

        float hsvValues[] = {0F, 0F, 0F};final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());

        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();

// add this before the while loop

        boolean senseblue = false;
        boolean sensered = false;

        while (opModeIsActive()) {

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),(int) (sensorColor.green() * SCALE_FACTOR),(int) (sensorColor.blue() * SCALE_FACTOR),hsvValues);

            //telemetry.addData("Distance (cm)",String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));

            if(sensorColor.red()>sensorColor.blue() && sensorColor.red()>sensorColor.green()){

                telemetry.addData("Red ", sensorColor.red());

            } else if(sensorColor.blue()>sensorColor.red() && sensorColor.blue()>sensorColor.green()){

                telemetry.addData("Blue ", sensorColor.blue());

            } else if(sensorColor.green()>sensorColor.blue() && sensorColor.green()>sensorColor.red()){

                telemetry.addData("Green", sensorColor.green());

            }

            telemetry.addData("Hue", hsvValues[0]);telemetry.update();

//Change to like this, this way sensblue will only update when it is false, and will remain true

            if (!senseblue) {

                senseblue = sensorColor.blue()>sensorColor.red() && sensorColor.blue()> sensorColor.green();

            }

            if (!sensered) {

                sensered = sensorColor.red()>sensorColor.red() && sensorColor.blue()> sensorColor.green();

            }



            if (!senseblue && !sensered){

                MotorFL.setPower(1.0);
                MotorFR.setPower(1.0);

            } else {

                MotorFL.setPower(0.0);
                MotorFR.setPower(0.0);

            }

        }
    }
    void Drive(float speed) {
        MotorFL.setPower(-speed);
        MotorBL.setPower(speed);
        MotorFR.setPower(speed);
        MotorBR.setPower(-speed);
    }

    void Turn(float speed){
        MotorFL.setPower(speed);
        MotorBL.setPower(-speed);
        MotorFR.setPower(speed);
        MotorBR.setPower(-speed);
    }

    void Strafe(float speed){
        MotorFL.setPower(speed);
        MotorBL.setPower(speed);
        MotorFR.setPower(speed);
        MotorBR.setPower(speed);
    }

}
