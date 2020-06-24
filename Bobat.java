package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Bobat {
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor rotMotor = null;
    public DcMotor tape = null;

    public Servo fServo2;
    public Servo armServo;
    public Servo phServo;


    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public Bobat() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        frontLeft = hwMap.get(DcMotor.class, "fL");
        frontRight = hwMap.get(DcMotor.class, "fR");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft = hwMap.get(DcMotor.class, "bL");
        backRight = hwMap.get(DcMotor.class, "bR");
//        fServo1 = hwMap.servo.get("fServo1");
        fServo2 = hwMap.servo.get("fServo2");
        phServo = hwMap.servo.get("ph");
        rotMotor = hwMap.get(DcMotor.class, "rot");
        tape = hwMap.dcMotor.get("tape");

        //  armServo = hwMap.servo.get("armServo");
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
