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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import ftclib.FtcOpMode;

import static org.firstinspires.ftc.teamcode.AngleMeasureHw.GYRO;
import static org.firstinspires.ftc.teamcode.AngleMeasureHw.IMU;

/**
 * This class will contain all methods for autonomous,
 * which can be used in different combinations for different opModes
 */

enum AllianceColor {RED, BLUE}
enum AngleMeasureHw {GYRO, IMU}

public class AutonomousActions {
    int   RED_THRESHOLD  = (100);
    int   BLUE_THRESHOLD = (60);
    double WALL_DISTANCE = 20;
    private static double gyroKp = 0.3;
    private static double gyroScale = 0.5;
    boolean firstTapeFound = false;
    FtcOpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    AllianceColor allianceColor;
    AngleMeasureHw angleMeasureHw;

    PickupHardware pickupHw = new PickupHardware();

    MecanumDriveBase mecanumDriveBase = new MecanumDriveBase();

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
    ColorSensor tapeSensorL = null;
    ColorSensor tapeSensorR = null;
    Servo jewelArm          = null;

    ModernRoboticsI2cGyro gyro  = null;
    BNO055IMU imu           = null;
    double angleZ;

    ModernRoboticsI2cRangeSensor leftRange = null;
    ModernRoboticsI2cRangeSensor rightRange = null;

    void init(FtcOpMode opMode, AllianceColor allianceColor, AngleMeasureHw angleMeasureHw) {
        initOpmode(opMode);
        initAlliance(allianceColor);
        initMecanum();
        initVuforia();
        initJewelHardware(angleMeasureHw);
        initGlyphHardware();
    }

    void initOpmode(FtcOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
    }

    void initAlliance(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    void initMecanum() {

        mecanumDriveBase.init(hardwareMap, opMode);

        mecanumDriveBase.leftFrontMotor.setInverted(true);
        mecanumDriveBase.rightFrontMotor.setInverted(false);
        mecanumDriveBase.leftBackMotor.setInverted(true);
        mecanumDriveBase.rightBackMotor.setInverted(false);

        mecanumDriveBase.leftFrontMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.rightFrontMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.leftBackMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.rightBackMotor.setBrakeModeEnabled(true);
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

        leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
        rightRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");

        pickupHw.init(hardwareMap);

        tapeSensorL = hardwareMap.get(ColorSensor.class, "bottomColorL");
        tapeSensorL.enableLed(true);
        tapeSensorR = hardwareMap.get(ColorSensor.class, "bottomColorR");
        tapeSensorR.enableLed(true);

    }

    // DON"T USE THIS YET
    public void glyphPickup() {
        leftServo = hardwareMap.get(Servo.class, "leftWheel");
        rightServo = hardwareMap.get(Servo.class, "rightWheel");

        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 0, 0);

        while (opMode.opModeIsActive() && touchSensor.getState() == false) ;

        mecanumDriveBase.mecanumDrive.stop();

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
        telemetry.addData("VuMark", "visible");
        telemetry.update();
    }

    boolean moveAwayFromColor() {
        return allianceColor == AllianceColor.BLUE && colorSensor.blue() > colorSensor.red()
                || allianceColor == AllianceColor.RED && colorSensor.red() > colorSensor.blue();
    }

    void place1stGlyph() { // TODO: place glyph in correct column

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addLine("VuMark Unknown");
        } if (vuMark == RelicRecoveryVuMark.LEFT) {
            telemetry.addLine("Glyph Left");
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 90, 0);
            opMode.sleep(1000);
            mecanumDriveBase.mecanumDrive.stop();
        } if (vuMark == RelicRecoveryVuMark.CENTER) {
            telemetry.addLine("Glyph Center");
        } if (vuMark == RelicRecoveryVuMark.RIGHT) {
            telemetry.addLine("Glyph Right");
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 270, 0);
            opMode.sleep(1000);
            mecanumDriveBase.mecanumDrive.stop();
        }
        telemetry.update();

        pickupHw.wristServo.setPosition(0);
        opMode.sleep(1000);
        pickupHw.wristServo.setPosition(0.5);

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
            mecanumDriveBase.turn(-10, 0.4);

            /*
            turnLeftWithoutAngle(0.5);
            while (opMode.opModeIsActive() && time.seconds() < timeLimit);
            mecanumDrive.stop();
            */
        }
        else {
            telemetry.addLine("Moving left"); // TODO: move arm to side with color sensor
            mecanumDriveBase.turn(10, 0.4);

            /*
            turnRightWithoutAngle(0.5);
            while (opMode.opModeIsActive() && time.seconds() < timeLimit);
            mecanumDrive.stop();
            */
        }
        telemetry.update();
        jewelArm.setPosition(1);
        mecanumDriveBase.turn(0, 0.3);
        opMode.sleep(500);
        //colorSensor.enableLed(false);
    }

    void driveToCryptobox() throws InterruptedException {

        /*
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 90, 0);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 270, 0);
        }
        */

        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(.8, 0, 0);
        opMode.sleep(650);


        while (opMode.opModeIsActive() && (Math.abs(getAngleY()) > 2 || Math.abs(getAngleZ()) > 2)) {
            telemetry.addData("Angle Y", getAngleY());
            telemetry.addData("Angle Z", getAngleZ());
            telemetry.update();
        }
        mecanumDriveBase.mecanumDrive.stop();
        //encoderDrive(0.5, 400, 2);

        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.turn(90);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.turn(270);
        }

        //tapeFinder();
        encoderDrive(0.6, 2400, 3);
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.6, 0, 0);
//        opMode.sleep(1000);
//        mecanumDriveBase.mecanumDrive.stop();
        ElapsedTime     runtime = new ElapsedTime();
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 270, 0);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 90, 0);
        }

        while (opMode.opModeIsActive() && (runtime.seconds() < .75)) {
            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.5, 0, 0);
        }
        mecanumDriveBase.mecanumDrive.stop();

        positionUsingTape();
        encoderDrive(0.3, 400, 1);
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 0, 0);
//        opMode.sleep(400);
//        mecanumDriveBase.mecanumDrive.stop();

//        ElapsedTime time = new ElapsedTime();
//        time.reset();
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.8, 0, 0);
//
//        while (opMode.opModeIsActive() && time.seconds() < 1) {
//            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
//            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
//
//        mecanumDriveBase.mecanumDrive.stop();
//
//        if (allianceColor == AllianceColor.BLUE) {
//            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 270, 0);
//        } else if (allianceColor == AllianceColor.RED) {
//            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 90, 0);
//        }
//
//        time.reset();
//        while (opMode.opModeIsActive() && time.seconds() < 1) {
//            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
//            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
//
//        mecanumDriveBase.mecanumDrive.stop();

        /*
        mecanumDrive.mecanumDrive_Polar(0.6, 90, 0);
        opMode.sleep(2000);
        mecanumDrive.stop();

        turn(90);
        */
    }

    void driveToCryptobox1() throws InterruptedException {

        /*
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 90, 0);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 270, 0);
        }
        */

        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(.8, 0, 0);
        opMode.sleep(650);


        while (opMode.opModeIsActive() && (Math.abs(getAngleY()) > 2 || Math.abs(getAngleZ()) > 2)) {
            telemetry.addData("Angle Y", getAngleY());
            telemetry.addData("Angle Z", getAngleZ());
            telemetry.update();
        }
        mecanumDriveBase.mecanumDrive.stop();
        //encoderDrive(0.5, 400, 2);

        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.turn(90);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.turn(270);
        }

        //tapeFinder();
        encoderColorDrive(0.6, 2500, 3);
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.6, 0, 0);
//        opMode.sleep(1000);
//        mecanumDriveBase.mecanumDrive.stop();
        if (!firstTapeFound) {
            ElapsedTime runtime = new ElapsedTime();
            if (allianceColor == AllianceColor.BLUE) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 270, 0);
            } else if (allianceColor == AllianceColor.RED) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 90, 0);
            }

            while (opMode.opModeIsActive() && (runtime.seconds() < 1)) {
                //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.5, 0, 0);
            }
            mecanumDriveBase.mecanumDrive.stop();
        }

        mecanumDriveBase.mecanumDrive.stop();
        opMode.sleep(500);
        positionUsingTape1();
        encoderDrive(0.3, 400, 1);
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 0, 0);
//        opMode.sleep(400);
//        mecanumDriveBase.mecanumDrive.stop();

//        ElapsedTime time = new ElapsedTime();
//        time.reset();
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.8, 0, 0);
//
//        while (opMode.opModeIsActive() && time.seconds() < 1) {
//            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
//            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
//
//        mecanumDriveBase.mecanumDrive.stop();
//
//        if (allianceColor == AllianceColor.BLUE) {
//            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 270, 0);
//        } else if (allianceColor == AllianceColor.RED) {
//            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 90, 0);
//        }
//
//        time.reset();
//        while (opMode.opModeIsActive() && time.seconds() < 1) {
//            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
//            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
//
//        mecanumDriveBase.mecanumDrive.stop();

        /*
        mecanumDrive.mecanumDrive_Polar(0.6, 90, 0);
        opMode.sleep(2000);
        mecanumDrive.stop();

        turn(90);
        */
    }

    void driveToCryptobox2() throws InterruptedException {

        /*
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 90, 0);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 270, 0);
        }
        */

        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(.8, 0, 0);
        opMode.sleep(650);


        while (opMode.opModeIsActive() && (Math.abs(getAngleY()) > 2 || Math.abs(getAngleZ()) > 2)) {
            telemetry.addData("Angle Y", getAngleY());
            telemetry.addData("Angle Z", getAngleZ());
            telemetry.update();
        }
        mecanumDriveBase.mecanumDrive.stop();
        //encoderDrive(0.5, 400, 2);

        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.turn(90);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.turn(270);
        }

        //tapeFinder();
        encoderColorDrive(0.6, 2500, 3);
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.6, 0, 0);
//        opMode.sleep(1000);
//        mecanumDriveBase.mecanumDrive.stop();
        ElapsedTime runtime = new ElapsedTime();
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 270, 0);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 90, 0);
        }

        while (opMode.opModeIsActive() && (runtime.seconds() < .5)) {
            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.5, 0, 0);
        }
        mecanumDriveBase.mecanumDrive.stop();

        opMode.sleep(500);
        positionUsingTape();
        encoderDrive(0.3, 400, 1);
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 0, 0);
//        opMode.sleep(400);
//        mecanumDriveBase.mecanumDrive.stop();

//        ElapsedTime time = new ElapsedTime();
//        time.reset();
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.8, 0, 0);
//
//        while (opMode.opModeIsActive() && time.seconds() < 1) {
//            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
//            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
//
//        mecanumDriveBase.mecanumDrive.stop();
//
//        if (allianceColor == AllianceColor.BLUE) {
//            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 270, 0);
//        } else if (allianceColor == AllianceColor.RED) {
//            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 90, 0);
//        }
//
//        time.reset();
//        while (opMode.opModeIsActive() && time.seconds() < 1) {
//            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
//            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
//
//        mecanumDriveBase.mecanumDrive.stop();

        /*
        mecanumDrive.mecanumDrive_Polar(0.6, 90, 0);
        opMode.sleep(2000);
        mecanumDrive.stop();

        turn(90);
        */
    }

    void rangeTurn() {
        if (rightRange.getDistance(DistanceUnit.CM) > leftRange.getDistance(DistanceUnit.CM)) {
            while (opMode.opModeIsActive() &&
                    Math.abs(rightRange.getDistance(DistanceUnit.CM) - leftRange.getDistance(DistanceUnit.CM)) > 2) {
                mecanumDriveBase.turnLeftWithoutAngle(0.7);
                telemetry.addData("Right Distance", rightRange.getDistance(DistanceUnit.CM));
                telemetry.addData("Left Distance", leftRange.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
        else if (rightRange.getDistance(DistanceUnit.CM) < leftRange.getDistance(DistanceUnit.CM)) {
            while (opMode.opModeIsActive() &&
                    Math.abs(rightRange.getDistance(DistanceUnit.CM) - leftRange.getDistance(DistanceUnit.CM)) > 2) {
                mecanumDriveBase.turnLeftWithoutAngle(0.7);
                telemetry.addData("Right Distance", rightRange.getDistance(DistanceUnit.CM));
                telemetry.addData("Left Distance", leftRange.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
        mecanumDriveBase.mecanumDrive.stop();
    }

    void distanceToWall() {
        while (opMode.opModeIsActive() &&
                (Math.abs(leftRange.getDistance(DistanceUnit.CM) - WALL_DISTANCE) > 2
                || Math.abs(rightRange.getDistance(DistanceUnit.CM) - WALL_DISTANCE) > 2)) {
            if (Math.abs(leftRange.getDistance(DistanceUnit.CM) - rightRange.getDistance(DistanceUnit.CM)) > 2) {
                rangeTurn();
            }
            else if (leftRange.getDistance(DistanceUnit.CM) > WALL_DISTANCE) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 0, 0);
            }
            else if (leftRange.getDistance(DistanceUnit.CM) < WALL_DISTANCE) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 180, 0);
            }
        }
        mecanumDriveBase.mecanumDrive.stop();
    }

    void distanceToWall1() throws InterruptedException {
        while (opMode.opModeIsActive() && Math.abs(leftRange.getDistance(DistanceUnit.CM) - WALL_DISTANCE) > 2) {
            if (getAngleX() > 3) {
                if (allianceColor == AllianceColor.BLUE) {
                    mecanumDriveBase.turn(90);
                }
                else if (allianceColor == AllianceColor.RED) {
                    mecanumDriveBase.turn(270);
                }
            }
            else if (leftRange.getDistance(DistanceUnit.CM) > WALL_DISTANCE) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 0, 0);
            }
            else if (leftRange.getDistance(DistanceUnit.CM) < WALL_DISTANCE) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 180, 0);
            }
        }
        mecanumDriveBase.mecanumDrive.stop();
    }

    void positionUsingTape() throws InterruptedException {
        //ElapsedTime time = new ElapsedTime();

        boolean current;   // Used to make sure only the 2nd tape is read and not the 1st
        boolean previous;
        boolean far = false; // Tells whether 1st sensor reaches 2nd tapee before 2nd sensor reaches 1st tape

        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 90, 0);
            while (opMode.opModeIsActive() && tapeSensorL.blue() < BLUE_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Blue", tapeSensorL.blue());
                telemetry.addData("Right Tape Sensor: Blue", tapeSensorR.blue());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
            }
            current = true;
            //time.reset();
            while (opMode.opModeIsActive() && tapeSensorR.blue() < BLUE_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Blue", tapeSensorL.blue());
                telemetry.addData("Right Tape Sensor: Blue", tapeSensorR.blue());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
                previous = current;
                current = tapeSensorL.blue() > BLUE_THRESHOLD;
                if (current && !previous) { // 2nd tape is only detected when 1st sensor goes from black to tape
                    far = true;
                }
            }
            mecanumDriveBase.mecanumDrive.stop();
            opMode.sleep(400);
            mecanumDriveBase.turn(90);
            if (far) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, -42, 0);
            }
            else {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, -222, 0);
            }
            while (opMode.opModeIsActive() && tapeSensorL.blue() < BLUE_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Blue", tapeSensorL.blue());
                telemetry.addData("Right Tape Sensor: Blue", tapeSensorR.blue());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
            }
        }
        else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 270, 0);
            while (opMode.opModeIsActive() && tapeSensorR.red() < RED_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Red", tapeSensorL.red());
                telemetry.addData("Right Tape Sensor: Red", tapeSensorR.red());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
            }
            current = true;
            //time.reset();
            while (opMode.opModeIsActive() && tapeSensorL.red() < RED_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Red", tapeSensorL.red());
                telemetry.addData("Right Tape Sensor: Red", tapeSensorR.red());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
                previous = current;
                current = tapeSensorR.red() > RED_THRESHOLD;
                if (current && !previous) { // 2nd tape is only detected when 1st sensor goes from black to tape
                    far = true;
                }
            }
            mecanumDriveBase.mecanumDrive.stop();
            opMode.sleep(400);
            mecanumDriveBase.turn(270);
            if (far) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 42, 0);
            }
            else {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 222, 0);
            }
            while (opMode.opModeIsActive() && tapeSensorR.red() < RED_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Red", tapeSensorL.red());
                telemetry.addData("Right Tape Sensor: Red", tapeSensorR.red());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
            }
        }
        mecanumDriveBase.mecanumDrive.stop();
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.turn(90);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.turn(270);
        }
    }

    void positionUsingTape1() throws InterruptedException {
        //ElapsedTime time = new ElapsedTime();

        boolean current;   // Used to make sure only the 2nd tape is read and not the 1st
        boolean previous;
        boolean far = false; // Tells whether 1st sensor reaches 2nd tapee before 2nd sensor reaches 1st tape

        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 90, 0);
            if (!firstTapeFound) {
                while (opMode.opModeIsActive() && tapeSensorL.blue() < BLUE_THRESHOLD) {
                    telemetry.addData("Left Tape Sensor: Blue", tapeSensorL.blue());
                    telemetry.addData("Right Tape Sensor: Blue", tapeSensorR.blue());
                    telemetry.addData("Boolean Far", far);
                    telemetry.update(); //Tells the intensity of the blue color we are looking for
                }
            }
            current = true;
            //time.reset();
            while (opMode.opModeIsActive() && tapeSensorR.blue() < BLUE_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Blue", tapeSensorL.blue());
                telemetry.addData("Right Tape Sensor: Blue", tapeSensorR.blue());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
                previous = current;
                current = tapeSensorL.blue() > BLUE_THRESHOLD;
                if (current && !previous) { // 2nd tape is only detected when 1st sensor goes from black to tape
                    far = true;
                }
            }
            mecanumDriveBase.mecanumDrive.stop();
            opMode.sleep(400);
            mecanumDriveBase.turn(90);
            if (far) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, -42, 0);
            }
            else {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, -222, 0);
            }
            while (opMode.opModeIsActive() && tapeSensorL.blue() < BLUE_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Blue", tapeSensorL.blue());
                telemetry.addData("Right Tape Sensor: Blue", tapeSensorR.blue());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
            }
        }
        else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 270, 0);
            if (!firstTapeFound) {
                while (opMode.opModeIsActive() && tapeSensorR.red() < RED_THRESHOLD) {
                    telemetry.addData("Left Tape Sensor: Red", tapeSensorL.red());
                    telemetry.addData("Right Tape Sensor: Red", tapeSensorR.red());
                    telemetry.addData("Boolean Far", far);
                    telemetry.update(); //Tells the intensity of the blue color we are looking for
                }
            }
            current = true;
            //time.reset();
            while (opMode.opModeIsActive() && tapeSensorL.red() < RED_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Red", tapeSensorL.red());
                telemetry.addData("Right Tape Sensor: Red", tapeSensorR.red());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
                previous = current;
                current = tapeSensorR.red() > RED_THRESHOLD;
                if (current && !previous) { // 2nd tape is only detected when 1st sensor goes from black to tape
                    far = true;
                }
            }
            mecanumDriveBase.mecanumDrive.stop();
            opMode.sleep(400);
            mecanumDriveBase.turn(270);
            if (far) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 42, 0);
            }
            else {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 222, 0);
            }
            while (opMode.opModeIsActive() && tapeSensorR.red() < RED_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Red", tapeSensorL.red());
                telemetry.addData("Right Tape Sensor: Red", tapeSensorR.red());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
            }
        }
        mecanumDriveBase.mecanumDrive.stop();
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.turn(90);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.turn(270);
        }
    }

    void tapeFinder() {
        /*
        mecanumDriveBase.leftFrontMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.rightFrontMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.leftBackMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.rightBackMotor.setBrakeModeEnabled(true);
        ElapsedTime elapsedTime = new ElapsedTime();
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_XPolar(0.7, 90, 0);
            while ((opMode.opModeIsActive() && elapsedTime.seconds() < 1) || (tapeSensorL.blue() < BLUE_THRESHOLD)) {

                telemetry.addData("Tape Sensor: Blue", tapeSensorL.blue());
                telemetry.update(); //Tells the intensity of the blue color we are looking for
            }
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_XPolar(0.7, 270, 0);
            while ((opMode.opModeIsActive() && elapsedTime.seconds() < 1) || (tapeSensorR.red() < RED_THRESHOLD)) {

                telemetry.addData("Tape Sensor: Red", tapeSensorR.red());
                telemetry.update(); //Tells the intensity of the red color we are looking for
            }
        }
        mecanumDriveBase.mecanumDrive.stop();
        //*/
        mecanumDriveBase.leftFrontMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.rightFrontMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.leftBackMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.rightBackMotor.setBrakeModeEnabled(true);
        mecanumDriveBase.mecanumDrive.mecanumDrive_XPolar(0.7, 0, 0);
        opMode.sleep(1700);
        if (allianceColor == AllianceColor.BLUE) {
            opMode.sleep(700);
        }
        mecanumDriveBase.mecanumDrive.mecanumDrive_XPolar(0, 0, 0);
    }

    void moveFWBW() {
        ElapsedTime     runtime = new ElapsedTime();

        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 180, 0);
        while (opMode.opModeIsActive() && (runtime.seconds() < 0.2)) {
            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.5, 0, 0);
        }
        mecanumDriveBase.mecanumDrive.stop();
        opMode.sleep(500);
        runtime.reset();
        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 180, 0);
        while (opMode.opModeIsActive() && (runtime.seconds() < 0.2)) {
            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(-0.5, 0, 0);
        }
        mecanumDriveBase.mecanumDrive.stop();
        runtime.reset();
        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 0, 0);
        while (opMode.opModeIsActive() && (runtime.seconds() < 0.4)) {
            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.5, 0, 0);
        }
        mecanumDriveBase.mecanumDrive.stop();
        runtime.reset();
        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 180, 0);
        while (opMode.opModeIsActive() && (runtime.seconds() < 0.4)) {
            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(-0.5, 0, 0);
        }
        mecanumDriveBase.mecanumDrive.stop();
        //mecanumMotors.mecanumDrive.stop();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
        //mecanumDriveBase.mecanumDrive.stop();
        //mecanumDriveBase.leftFrontMotor.setBrakeModeEnabled(false);
        //mecanumDriveBase.rightFrontMotor.setBrakeModeEnabled(false);
        //mecanumDriveBase.leftBackMotor.setBrakeModeEnabled(false);
        //mecanumDriveBase.rightBackMotor.setBrakeModeEnabled(false);
    }

    void ejectGlyph() {
        ElapsedTime time = new ElapsedTime();
        pickupHw.leftServo.setPosition(-1.0);
        pickupHw.rightServo.setPosition(1.0);
        while (opMode.opModeIsActive() && time.seconds() < 1.5);
        pickupHw.leftServo.setPosition(0.52);
        pickupHw.rightServo.setPosition(0.5);
    }

    void encoderColorDrive(double speed,
                             double encoderCounts,
                             double timeoutS) {
        int startPosL;
        int startPosR;
        int newLeftTarget;
        int newRightTarget;
        DcMotor motorL = mecanumDriveBase.leftBackMotor.motor;
        DcMotor motorR = mecanumDriveBase.rightBackMotor.motor;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            int minDist = 1900;
            startPosL = motorL.getCurrentPosition();
            startPosR = motorR.getTargetPosition();
            // Determine new target position, and pass to motor controller
            newLeftTarget = motorL.getCurrentPosition() + (int)(encoderCounts);
            newRightTarget = motorR.getTargetPosition() + (int)(encoderCounts);
            motorL.setTargetPosition(newLeftTarget);
            motorR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            mecanumDriveBase.setBrakeModeEnabled(true);
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(speed, 0, 0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorL.isBusy()) && motorR.isBusy()) {

                if (checkCloserColor()) {
                    firstTapeFound = true;
                }

                if (firstTapeFound
                        && motorL.getCurrentPosition() - startPosL > minDist
                        && motorR.getCurrentPosition() - startPosR > minDist) {
                    mecanumDriveBase.mecanumDrive.stop();
                    motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    return;
                }
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorL.getCurrentPosition(),
                        motorR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            mecanumDriveBase.mecanumDrive.stop();

            // Turn off RUN_TO_POSITION
            motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    private boolean checkCloserColor() {
        return (allianceColor == AllianceColor.BLUE && tapeSensorL.blue() > BLUE_THRESHOLD
        || allianceColor == AllianceColor.RED && tapeSensorR.red() > RED_THRESHOLD);
    }

    public void encoderDrive(double speed,
                             double encoderCounts,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        DcMotor motorL = mecanumDriveBase.leftBackMotor.motor;
        DcMotor motorR = mecanumDriveBase.rightBackMotor.motor;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorL.getCurrentPosition() + (int)(encoderCounts);
            newRightTarget = motorR.getTargetPosition() + (int)(encoderCounts);
            motorL.setTargetPosition(newLeftTarget);
            motorR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            mecanumDriveBase.setBrakeModeEnabled(true);
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(speed, 0, 0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorL.isBusy()) && motorR.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorL.getCurrentPosition(),
                        motorR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            mecanumDriveBase.mecanumDrive.stop();

            // Turn off RUN_TO_POSITION
            motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
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
