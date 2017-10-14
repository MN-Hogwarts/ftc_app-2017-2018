/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
import swlib.SWIMUGyro;
import trclib.TrcDriveBase;

/**
 * This class will contain all methods for autonomous,
 * which can be used in different combinations for different opModes
 */

enum AllianceColor {RED, BLUE}

public class AutonomousActions {

    FtcOpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    AllianceColor allianceColor;

    FtcDcMotor leftFrontMotor   = null;
    FtcDcMotor rightFrontMotor  = null;
    FtcDcMotor leftBackMotor    = null;
    FtcDcMotor rightBackMotor   = null;
    TrcDriveBase mecanumDrive   = null;
    DigitalChannel touchSensor  = null;


    Servo leftServo;
    Servo rightServo;

    public static final String TAG = "Testing Vuforia";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;

    ColorSensor colorSensor = null;
    Servo jewelArm          = null;


    ModernRoboticsI2cGyro gyro  = null;
    BNO055IMU imu           = null;
    double angleZ;

    void initOpmode(FtcOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = hardwareMap;
    }

    void initAlliance(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    void initVuforia() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AfDQ5Or/////AAAAGe1Hp0y47kyQgLQeC4/tzkl1arRGdELMrgm+iq9KNr01oclHMaAkK+xIAxV0+ftJtwS89VFPjk9tda4h94V7QUzvWheuhqfsm/Sok3iFV4vZUq+dFVXL2FcCzhTZHXWcIEtWNqTTKgwsWF7CU/BfAJBb0AtOHUGyD0nT87dO2lzE3oe5UN7naKFaYmYXN/yoktKZEYD4pUxbjxGqiVIsXA7+BECGs/kxWsVuF0eWwbBChLYngPsJqIE1g8noobXGCTs+u3mFI41GOwpkDskq7TU3YjdUxkTC6fKbESLQLh+0SpOYkiVLtb20o3qU9OaCs7nCEJyko7Vxs/rSjO+IhMzo0M2PUhiFKNxpLwrlf4dE";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

    }

    void initJewelHardware() {
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        jewelArm = hardwareMap.get(Servo.class, "jewel_arm");
        jewelArm.setPosition(0.1);
        leftFrontMotor = new FtcDcMotor("left_front");
        rightFrontMotor = new FtcDcMotor("right_front");
        leftBackMotor = new FtcDcMotor("left_rear");
        rightBackMotor = new FtcDcMotor("right_rear");
        leftFrontMotor.setInverted(false);
        rightFrontMotor.setInverted(true);
        leftBackMotor.setInverted(false);
        rightBackMotor.setInverted(true);

        mecanumDrive = new TrcDriveBase(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
    }

    public void moveSirvo(){

    }


    public void glyphPickup() {
        leftServo = hardwareMap.get(Servo.class, "leftWheel");
        rightServo = hardwareMap.get(Servo.class, "rightWheel");

        leftFrontMotor.setPower(0.5);
        rightFrontMotor.setPower(0.5);
        leftBackMotor.setPower(0.5);
        rightBackMotor.setPower(0.5);

        while (opMode.opModeIsActive() && touchSensor.getState() == false) ;
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }
// find the Target Device section under the General tab on the Android Application page
    public void pictographID() {

        initVuforia();

        relicTrackables.activate();
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (opMode.opModeIsActive() && vuMark == RelicRecoveryVuMark.UNKNOWN && time.seconds() <= 8) { // 8 second timeout while looking for pictographs

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            opMode.telemetry.addData("VuMark", "not visible");
            opMode.telemetry.update();
        }
    }

    void jewelColor() throws InterruptedException {

        double timeLimit = 1.0;
        jewelArm.setPosition(0.84);
        colorSensor.enableLed(true);
        telemetry.addLine("Alliance Color: " + allianceColor);
        ElapsedTime time = new ElapsedTime();

        opMode.sleep(700);
        if (moveAwayFromColor()) {
            telemetry.addLine("Moving left"); // TODO: move arm to side without color sensor
            turn(10);

            /*
            turnLeftWithoutAngle(0.5);
            while (opMode.opModeIsActive() && time.seconds() < timeLimit);
            mecanumDrive.stop();
            */
        }
        else {
            telemetry.addLine("Moving right"); // TODO: move arm to side with color sensor
            turn(-10);

            /*
            turnRightWithoutAngle(0.5);
            while (opMode.opModeIsActive() && time.seconds() < timeLimit);
            mecanumDrive.stop();
            */
        }
        telemetry.update();
        turn(0);
        //colorSensor.enableLed(false);
    }

    boolean moveAwayFromColor() {
        return allianceColor == AllianceColor.BLUE && colorSensor.blue() > colorSensor.red()
                || allianceColor == AllianceColor.RED && colorSensor.red() > colorSensor.blue();
    }

    void place1stGlyph() { // TODO: place glyph in correct column
        if (vuMark == RelicRecoveryVuMark.UNKNOWN)
            opMode.telemetry.addLine("VuMark Unknown");
        if (vuMark == RelicRecoveryVuMark.LEFT)
            opMode.telemetry.addLine("Glyph Left");
        if (vuMark == RelicRecoveryVuMark.CENTER)
            opMode.telemetry.addLine("Glyph Center");
        if (vuMark == RelicRecoveryVuMark.RIGHT)
            opMode.telemetry.addLine("Glyph Right");
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void turn(int turnAngle) throws InterruptedException {

        // leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // int leftPos = leftMotor.getCurrentPosition();
        // int rightPos = rightMotor.getCurrentPosition();

        double startAngle = getHeading();
        angleZ = getHeading();

        double angDiff = (turnAngle - angleZ) % 360;
        if (360 - Math.abs(angDiff) < Math.abs(angDiff))
            angDiff = -(360 * Math.signum(angDiff) - angDiff);

        telemetry.log().add("Angle Difference: " + angDiff);
        telemetry.update();

        if (angDiff < 0) { //turns right
            //leftMotor.setPower(APPROACH_SPEED * .6 );
            //rightMotor.setPower(-APPROACH_SPEED * .6);

            while (opMode.opModeIsActive() && angDiff < 0) {

                angleZ = getHeading();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                telemetry.addData("Angle", angleZ);
                telemetry.addData("Difference", angDiff);
                telemetry.update();

                turnRightWithoutAngle(turnPower(angDiff));
                /*
                leftFrontMotor.setPower(turnPower(angDiff));
                rightFrontMotor.setPower(-turnPower(angDiff));
                leftBackMotor.setPower(turnPower(angDiff));
                rightBackMotor.setPower(-turnPower(angDiff));
                */
                telemetry.addData("Power", leftFrontMotor.getPower());

                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, -90, false);
                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, angDiff);

                /* if (leftMotor.getCurrentPosition() - 100 > leftPos
                        && rightMotor.getCurrentPosition() + 100 < rightPos
                        && IMUheading() == startAngle) {
                    resetIMuandPos(leftPos, rightPos);
                } */

                opMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        } else if (angDiff > 0) { //turns left
            //leftMotor.setPower(-APPROACH_SPEED);
            //rightMotor.setPower(APPROACH_SPEED);

            while (opMode.opModeIsActive() && angDiff > 0) {

                angleZ = getHeading();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                telemetry.addData("Angle", angleZ);
                telemetry.addData("Difference", angDiff);
                telemetry.update();

                turnLeftWithoutAngle(turnPower(angDiff));
                /*
                leftFrontMotor.setPower(turnPower(angDiff));
                rightFrontMotor.setPower(-turnPower(angDiff));
                leftBackMotor.setPower(turnPower(angDiff));
                rightBackMotor.setPower(-turnPower(angDiff));
                telemetry.addData("Power", leftFrontMotor.getPower());
                */

                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, 90, false);
                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, angDiff);

                /* if (leftMotor.getCurrentPosition() + 100 < leftPos
                        && rightMotor.getCurrentPosition() - 100 > rightPos
                        && IMUheading() == startAngle) {
                    resetIMuandPos(leftPos, rightPos);
                } */

                opMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        }

        mecanumDrive.stop();
        /*
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        */
    }

    private void turnLeftWithoutAngle(double power) {
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(power);
    }

    private void turnRightWithoutAngle(double power) {
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(-power);
    }

    private double turnPower(double difference) {
        if (Math.abs(difference) < 20) {
            return 0.1;
        } else if (Math.abs(difference) < 45) {
            return 0.15;
        } else if (Math.abs(difference) < 90) {
            return 0.3;
        } else return 0.4;
    }

    double getHeading() {
        if (gyro != null) {
            return gyro.getHeading();
        }
        else if (imu != null) {
            return imu.getAngularOrientation().thirdAngle;
        }
        else {
            return 0;
        }
    }
}
