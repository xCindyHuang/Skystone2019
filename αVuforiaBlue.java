/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

@Autonomous(name="VuforiaBlue", group ="Concept")
public class αVuforiaBlue extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
     //       " Ab7HqNX/////AAABmdnDkBLWGEnOg5hQXSh0ndkwu8q4ShVgPUGXJf/bzTucWLiBDJ/aVPbCgtn5kvY1ZqGJxMBSWqfhGulHJcb/t/p6ehxUdvyv+TrSiviuZl3MRM4YhocbdrMTI5NQlZkSpfsRFuSGenl02aP/jwmOMKszvkMiHHNanzJboEmm4BPdilaKWD+5KwA2lhQsP09QaS6ntqKo2PGSiA/K/gFGqJeAI+sKDkh3hwOHpCQt4tkBQE0tsm2XqRr+NWdPKmd4jlFM5BuqiyhxEXZr0j4CESbPvRUJwniQtSrQM4Ig3XXODWaBNW+vYJcra5xF4FHfS6wHNfc9X9C6d98Mce7A+Bu2+WhKBfR6jmY9HdhOSvyy ";"";
        "AfSJVqX/////AAABmYrY5rPR40ygk9uCQXjkV84G7sIxetH9CzdjPTRspVI52DgP9k6fuaeyn7+Pem6qdohAgWrLeggF2cjkidvJjk3kUp8HYgjzxwz++QabFEJIqFoUTp8IPURUw3Ng5Q8IbKX6GGoctq4UXvEUnBmTmdmU+LLDZMSIQQ6Hu6stbREiDrmV0qSs/vQNtFKXbkGu7tesFMtnXimTH6DWhFCQvNJt43JqBtAJxNbIS4NXAEqyw1OnPajb0TJMyzFASN8QlhQdYg3Ey8KX8nGSvZymc5X9Bb2lF0elTuQ/3beQV0lbxU7z0h1TrpP3DhX4FYsJV8whkqevFGMFQBvmexvR7UVOBChShR/mKkRrX3AMIurL";
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final float stoneZ = 2.00f * mmPerInch;


    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;
    private static final float bridgeRotZ = 180;
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.4;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;



    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    Bobat robot = new Bobat();
    private ElapsedTime runtime = new ElapsedTime();
    @Override public void runOpMode() {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        robot.fServo2.setPosition(0.0);
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT     = 0;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


        targetsSkyStone.activate();


    //    com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);
        com.vuforia.CameraDevice.getInstance().setField("opti-zoom", "opti-zoom-on");
        com.vuforia.CameraDevice.getInstance().setField("zoom","44");
        waitForStart();
        runtime.reset();
        boolean sky = false;
        double startTime = runtime.seconds();
        while (!isStopRequested() && runtime.seconds() - startTime < 3f) {

            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                double yPosition = translation.get(1) / mmPerInch;
                    if (yPosition < 0) {//left
                        sky = true;
                        blrDriveSafety(0.3, 6);
                        encodeDrive(0.3, 30, -30, -30, 30);
                        encodeDrive(0.3, -22, 22, -22, 22);
                        blrDriveSafety(0.4, -17);
                        robot.fServo2.setPosition(0.4);
                        sleep(500);
                        blrDriveSafety(0.5, 12);
                        encodeDrive(0.3, -22, 22, -22, 22);
                        robot.fServo2.setPosition(0.0);
                        blrDriveSafety(0.6, -50);

                        blrDriveSafety(0.3, 5);
                        encodeDrive(0.5, 44, -44, 44, -44);
                        runtime.reset();
                        while (opModeIsActive() && (runtime.seconds() < 1.5f)) {
                            robot.tape.setPower(1.0);
                        }
                        break;
                    } else if (yPosition > 0) {
                        sky = true;
                        blrDriveSafety(0.3, -3);
                        encodeDrive(0.3, 5, -5, -5, 5);
                        encodeDrive(0.3, -22, 22, -22, 22);
                        blrDriveSafety(0.3, -38);
                        robot.fServo2.setPosition(0.4);
                        sleep(500);
                        blrDriveSafety(0.5, 20);
                        encodeDrive(0.3, -22, 22, -22, 22);
                        robot.fServo2.setPosition(0.0);
                        blrDriveSafety(0.5, -60);//100

                        blrDriveSafety(0.3, 5);
                        encodeDrive(0.5, 44, -44, 44, -44);
                        runtime.reset();
                        while (opModeIsActive() && (runtime.seconds() < 1.5f)) {
                            robot.tape.setPower(1.0);
                        }
                        break;
                    }




                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
         if (sky==false) {
             blrDriveSafety(0.3, -8);
             encodeDrive(0.3, 7, -7, -7, 7);
             encodeDrive(0.3, -22, 22, -22, 22);
             blrDriveSafety(0.3, -38);
             robot.fServo2.setPosition(0.4);
             sleep(400);
             blrDriveSafety(0.5, 20);
             encodeDrive(0.3, -21, 21, -21, 21);
             robot.fServo2.setPosition(0.0);
             blrDriveSafety(0.8, -70);//110

             blrDriveSafety(0.3, 5);
             encodeDrive(0.5, 44, -44, 44, -44);
             runtime.reset();
             while (opModeIsActive() && (runtime.seconds() < 3.0f)) {//2.5f
                 robot.tape.setPower(1.0);
             }
         }
        targetsSkyStone.deactivate();
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
            robot.frontLeft.setTargetPosition(0);
            robot.frontRight.setTargetPosition(0);
            robot.backLeft.setTargetPosition(0);
            robot.backRight.setTargetPosition(0);
            robot.frontLeft.setTargetPosition(newFL);
            robot.frontRight.setTargetPosition(newFR);
            robot.backLeft.setTargetPosition(newBL);
            robot.backRight.setTargetPosition(newBR);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setPower(power);
            robot.frontRight.setPower(power);
            robot.backLeft.setPower(power);
            robot.backRight.setPower(power);

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 7 &&
                    robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() &&
                    robot.backRight.isBusy()
            ) {
//                telemetry.addData("FrontLeft Location:", "%7d", robot.frontLeft.getCurrentPosition());
//                telemetry.addData("FrontRight Location:", "%7d", robot.frontRight.getCurrentPosition());
//                telemetry.addData("BackLeft Location:", "%7d", (robot.backLeft.getCurrentPosition()));
//                telemetry.addData("BackRight Location:", "%7d", (robot.backRight.getCurrentPosition()));
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
            runtime.reset();
            while(runtime.seconds() < 0.7f)
            {        }
    }}
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
            robot.backLeft.setTargetPosition(0);
            robot.backRight.setTargetPosition(0);
            robot.backLeft.setTargetPosition(newBL);
            robot.backRight.setTargetPosition(newBR);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            runtime.reset();
            while(runtime.seconds() < 1.5f){

            }
        }
    }
    public void Drive (float speed){
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(speed);
        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(speed);
    }
}
