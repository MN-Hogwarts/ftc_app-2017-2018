package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
import swlib.SwDriveBase;

import static org.firstinspires.ftc.teamcode.AngleMeasureHw.GYRO;
import static org.firstinspires.ftc.teamcode.AngleMeasureHw.IMU;

/**
 * This class will contain all methods for autonomous,
 * which can be used in different combinations for different opModes
 */

enum AllianceColor {RED, BLUE}
enum AngleMeasureHw {GYRO, IMU}

public class AutonomousActions {
    int   RED_THRESHOLD  = (9);
    int   BLUE_THRESHOLD = (9);

    FtcOpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    AllianceColor allianceColor;
    AngleMeasureHw angleMeasureHw;

    PickupHardware pickupHw = new PickupHardware();

    MecanumMotors mecanumMotors = new MecanumMotors();

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
    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    ColorSensor colorSensor = null;
    ColorSensor tapeSensor = null;
    Servo jewelArm          = null;


    ModernRoboticsI2cGyro gyro  = null;
    BNO055IMU imu           = null;
    double angleZ;

    ModernRoboticsI2cRangeSensor leftRange = null;
    ModernRoboticsI2cRangeSensor rightRange = null;

    void initOpmode(FtcOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = hardwareMap;
    }

    void initAlliance(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    void initMecanum() {

        mecanumMotors.init(hardwareMap);
        telemetry.addLine("2A");
        telemetry.update();


        mecanumMotors.leftFrontMotor.setInverted(true);
        mecanumMotors.rightFrontMotor.setInverted(false);
        mecanumMotors.leftBackMotor.setInverted(true);
        mecanumMotors.rightBackMotor.setInverted(false);

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

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    void initJewelHardware(AngleMeasureHw angleMeasureHw) {

        this.angleMeasureHw = angleMeasureHw;

        if (this.angleMeasureHw == GYRO) {
            gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        }
        else {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        jewelArm = hardwareMap.get(Servo.class, "jewelArm");
        jewelArm.setPosition(1);

    }

    void initGlyphHardware() {

        //leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range");
        //rightRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_range");

        pickupHw.init(hardwareMap);

        tapeSensor = hardwareMap.get(ColorSensor.class, "bottomColor");
        tapeSensor.enableLed(true);

    }

    // DON"T USE THIS YET
    public void glyphPickup() {
        leftServo = hardwareMap.get(Servo.class, "leftWheel");
        rightServo = hardwareMap.get(Servo.class, "rightWheel");

        mecanumMotors.mecanumDrive.mecanumDrive_BoxPolar(0.5, 0, 0);

        while (opMode.opModeIsActive() && touchSensor.getState() == false) ;

        mecanumMotors.mecanumDrive.stop();

        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    public void pictographID() {

        //initVuforia();

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
            telemetry.addData("VuMark", "not visible");
            telemetry.addData("Time", time.seconds());
            telemetry.update();
        }
    }

    boolean moveAwayFromColor() {
        return allianceColor == AllianceColor.BLUE && colorSensor.blue() > colorSensor.red()
                || allianceColor == AllianceColor.RED && colorSensor.red() > colorSensor.blue();
    }

    void place1stGlyph() { // TODO: place glyph in correct column
        if (vuMark == RelicRecoveryVuMark.UNKNOWN)
            telemetry.addLine("VuMark Unknown");
        if (vuMark == RelicRecoveryVuMark.LEFT)
            telemetry.addLine("Glyph Left");
        if (vuMark == RelicRecoveryVuMark.CENTER)
            telemetry.addLine("Glyph Center");
        if (vuMark == RelicRecoveryVuMark.RIGHT)
            telemetry.addLine("Glyph Right");
    }

    void jewelColor() throws InterruptedException {

        double timeLimit = 1.0;
        jewelArm.setPosition(0.4);
        colorSensor.enableLed(true);
        telemetry.addLine("Alliance Color: " + allianceColor);
        ElapsedTime time = new ElapsedTime();

        opMode.sleep(1000);
        /*
        while (opMode.opModeIsActive()) {
            telemetry.addData("Color Sensor blue", colorSensor.blue());
            telemetry.addData("Color Sensor red", colorSensor.red());
            telemetry.log().add("Move away: " + moveAwayFromColor());
            telemetry.update();
        }
        */

        if (moveAwayFromColor()) {
            telemetry.addLine("Moving right"); // TODO: move arm to side without color sensor
            turn(-10, 0.4);

            /*
            turnLeftWithoutAngle(0.5);
            while (opMode.opModeIsActive() && time.seconds() < timeLimit);
            mecanumDrive.stop();
            */
        }
        else {
            telemetry.addLine("Moving left"); // TODO: move arm to side with color sensor
            turn(10, 0.4);

            /*
            turnRightWithoutAngle(0.5);
            while (opMode.opModeIsActive() && time.seconds() < timeLimit);
            mecanumDrive.stop();
            */
        }
        telemetry.update();
        jewelArm.setPosition(1);
        turn(0);
        //colorSensor.enableLed(false);
    }

    void driveToCryptobox() throws InterruptedException {

        if (allianceColor == AllianceColor.BLUE) {
            turn(90);
        } else if (allianceColor == AllianceColor.RED) {
            turn(270);
        }

        ElapsedTime time = new ElapsedTime();
        time.reset();
        mecanumMotors.mecanumDrive.mecanumDrive_BoxPolar(0.8, 0, 0);

        while (opMode.opModeIsActive() && time.seconds() < 1) {
            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        mecanumMotors.mecanumDrive.stop();

        if (allianceColor == AllianceColor.BLUE) {
            mecanumMotors.mecanumDrive.mecanumDrive_BoxPolar(1, 270, 0);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumMotors.mecanumDrive.mecanumDrive_BoxPolar(1, 90, 0);
        }

        time.reset();
        while (opMode.opModeIsActive() && time.seconds() < 1) {
            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        mecanumMotors.mecanumDrive.stop();

        /*
        mecanumDrive.mecanumDrive_Polar(0.6, 90, 0);
        opMode.sleep(2000);
        mecanumDrive.stop();

        turn(90);
        */
    }

    void tapeFinder() {
        mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.7, 0, 0);
        while (opMode.opModeIsActive() && (tapeSensor.red() < 9)) {

            telemetry.addData("Tape Sensor: Red", tapeSensor.red());
            telemetry.update(); //Tells the intensity of the color we are looking for
        }
        mecanumMotors.mecanumDrive.stop();
    }

    void ejectGlyph() {
        ElapsedTime time = new ElapsedTime();
        pickupHw.leftServo.setPosition(-1.0);
        pickupHw.rightServo.setPosition(1.0);
        while (opMode.opModeIsActive() && time.seconds() < 2);
        pickupHw.leftServo.setPosition(0.52);
        pickupHw.rightServo.setPosition(0.5);
    }

    public void turn(int turnAngle, double power) throws InterruptedException {

        // leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // int leftPos = leftMotor.getCurrentPosition();
        // int rightPos = rightMotor.getCurrentPosition();

        double startAngle = getAngleX();
        angleZ = getAngleX();

        double angDiff = (turnAngle - angleZ) % 360;
        if (360 - Math.abs(angDiff) < Math.abs(angDiff))
            angDiff = -(360 * Math.signum(angDiff) - angDiff);

        telemetry.log().add("Angle Difference: " + angDiff);
        telemetry.update();

        if (angDiff < 0) { //turns right
            //leftMotor.setPower(APPROACH_SPEED * .6 );
            //rightMotor.setPower(-APPROACH_SPEED * .6);

            while (opMode.opModeIsActive() && angDiff < 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                telemetry.addData("Angle", angleZ);
                telemetry.addData("Difference", angDiff);
                telemetry.update();

                turnRightWithoutAngle(power);
                /*
                leftFrontMotor.setPower(turnPower(angDiff));
                rightFrontMotor.setPower(-turnPower(angDiff));
                leftBackMotor.setPower(turnPower(angDiff));
                rightBackMotor.setPower(-turnPower(angDiff));
                */
                telemetry.addData("Power", mecanumMotors.leftFrontMotor.getPower());

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

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                telemetry.addData("Angle", angleZ);
                telemetry.addData("Difference", angDiff);
                telemetry.update();

                turnLeftWithoutAngle(power);
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

        mecanumMotors.mecanumDrive.stop();
        /*
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        */
    }

    public void turn(int turnAngle) throws InterruptedException {

        // leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // int leftPos = leftMotor.getCurrentPosition();
        // int rightPos = rightMotor.getCurrentPosition();

        double startAngle = getAngleX();
        angleZ = getAngleX();

        double angDiff = (turnAngle - angleZ) % 360;
        if (360 - Math.abs(angDiff) < Math.abs(angDiff))
            angDiff = -(360 * Math.signum(angDiff) - angDiff);

        telemetry.log().add("Angle Difference: " + angDiff);
        telemetry.update();

        if (angDiff < 0) { //turns right
            //leftMotor.setPower(APPROACH_SPEED * .6 );
            //rightMotor.setPower(-APPROACH_SPEED * .6);

            while (opMode.opModeIsActive() && angDiff < 0) {

                angleZ = getAngleX();
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
                telemetry.addData("Power", mecanumMotors.leftFrontMotor.getPower());

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

                angleZ = getAngleX();
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

        mecanumMotors.mecanumDrive.stop();
        /*
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        */
    }

    private void turnLeftWithoutAngle(double power) {
        mecanumMotors.leftFrontMotor.setPower(-power);
        mecanumMotors.rightFrontMotor.setPower(power);
        mecanumMotors.leftBackMotor.setPower(-power);
        mecanumMotors.rightBackMotor.setPower(power);
    }

    private void turnRightWithoutAngle(double power) {
        mecanumMotors.leftFrontMotor.setPower(power);
        mecanumMotors.rightFrontMotor.setPower(-power);
        mecanumMotors.leftBackMotor.setPower(power);
        mecanumMotors.rightBackMotor.setPower(-power);
    }

    private double turnPower(double difference) {
        if (Math.abs(difference) < 20) {
            //return 0.15;
            return 0.45;
        } else if (Math.abs(difference) < 45) {
            //return 0.3;
            return 0.6;
        } else if (Math.abs(difference) < 90) {
            return 0.7;
        } else return 0.8;
    }

    double getAngleX() {
        if (angleMeasureHw == IMU) {
            return imu.getAngularOrientation().firstAngle;
        }
        else {
            return 0;
        }
    }

    double getAngleY() {
        if (angleMeasureHw == IMU) {
            return imu.getAngularOrientation().secondAngle;
        }
        else {
            return 0;
        }
    }

    double getAngleZ() {
        if (angleMeasureHw == GYRO) {
            return gyro.getHeading();
        }
        else if (angleMeasureHw == IMU) {
            return imu.getAngularOrientation().thirdAngle;
        }
        else {
            return 0;
        }
    }
}
