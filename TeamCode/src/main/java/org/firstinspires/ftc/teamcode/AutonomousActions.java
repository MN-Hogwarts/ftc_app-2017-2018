package org.firstinspires.ftc.teamcode;

import android.util.Log;

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
import org.firstinspires.ftc.teamcode.AutoOptions.AllianceColor;
import org.firstinspires.ftc.teamcode.AutoOptions.AngleMeasureHw;
import org.firstinspires.ftc.teamcode.VisitorPattern.VisitableActions;

import java.util.HashMap;

import ftclib.FtcOpMode;

import static org.firstinspires.ftc.teamcode.AutoOptions.AngleMeasureHw.GYRO;
import static org.firstinspires.ftc.teamcode.AutoOptions.AngleMeasureHw.IMU;

public class AutonomousActions implements VisitableActions{
    int   RED_THRESHOLD  = (110);
    int   BLUE_THRESHOLD = (90);
    double WALL_DISTANCE = 20;
    private static double gyroKp = 0.3;
    private static double gyroScale = 0.5;
    boolean firstTapeFound = false;
    FtcOpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    AllianceColor allianceColor = null;
    AngleMeasureHw angleMeasureHw;

    PickupHardware pickupHw = new PickupHardware();

    MecanumDriveBase mecanumDriveBase = new MecanumDriveBase();

    DigitalChannel touchSensor  = null;

    Servo leftServo;
    Servo rightServo;

    //public static final String TAG = "Testing Vuforia";
    public static final String TAG = "Autonomous";

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

    ModernRoboticsI2cRangeSensor leftRange  = null;
    ModernRoboticsI2cRangeSensor rightRange = null;

    ColorSensor innerColor  = null; // Positions of color sensors depends on alliance
    ColorSensor outerColor  = null; // Uses positioning when glyph pickup is facing back cryptobox
    int backCryptoboxAngle;         // Angle of robot when turned toward back cryptobox after starting on back balancing stone
    double innerDirection;             // Robot strafes at this angle to go toward center from backCryptoboxAngle
    double outerDirection;             // Robot strafes at this angle to go away from center from backCryptoboxAngle

    HashMap<String, Boolean> tapeMap = new HashMap<String, Boolean>();
    String outSensOutTape = "outerSensorOuterTape";
    String outSensInTape = "outerSensorInnerTape";
    String inSensInTape = "innerSensorInnerTape";

    boolean insideColorSensed   = false;
    boolean outsideColorSensed  = false;

    void init(FtcOpMode opMode, AllianceColor allianceColor, AngleMeasureHw angleMeasureHw) {
        initOpmode(opMode);
        initMecanum();
        initVuforia();
        initJewelHardware(angleMeasureHw);
        initGlyphHardware();
        initAlliance();
        initAlliance(allianceColor);
    }

    void initOpmode(FtcOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        Log.d(TAG, "initOpmode: initialized");
    }

    void initAlliance() {
        telemetry.update();
        if(tapeSensorR.red() > RED_THRESHOLD && tapeSensorL.red() > RED_THRESHOLD) {
            AllianceColor allianceColor1 = allianceColor.RED;
            allianceColor = allianceColor1;
        }
        else if (tapeSensorR.blue() > BLUE_THRESHOLD && tapeSensorL.blue() > BLUE_THRESHOLD){
            AllianceColor allianceColor1 = allianceColor.BLUE;
            allianceColor = allianceColor1;
        }

        initColorSensorSides();

        Log.d(TAG, "initAlliance: initialized");
    }

    void initAlliance(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;

        if (tapeSensorR != null && tapeSensorL != null) {
            initColorSensorSides();
        }
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

        mecanumDriveBase.setAllMotorsRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Log.d(TAG, "initMecanum: drivebase initialized");
    }

    void initVuforia() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        Log.d(TAG, "initVuforia: vuforia started");

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

        Log.d(TAG, "initVuforia: camera initialized");

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        Log.d(TAG, "initVuforia: data sets loaded");
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
            Log.d(TAG, "initJewelHardware: IMU initialized");
        }

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        Log.d(TAG, "initJewelHardware: color sensor initialized");
        jewelArm = hardwareMap.get(Servo.class, "jewelArm");
        jewelArm.setPosition(1);
        Log.d(TAG, "initJewelHardware: jewel arm servo initialized");

    }

    void initGlyphHardware() {

        leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
        rightRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");

        Log.d(TAG, "initGlyphHardware: range sensors initialized");

        pickupHw.init(hardwareMap);
        pickupHw.rightHinge.setPosition(0.2);

        Log.d(TAG, "initGlyphHardware: pickup hardware initialized");

        tapeSensorL = hardwareMap.get(ColorSensor.class, "bottomColorL");
        tapeSensorL.enableLed(true);
        tapeSensorR = hardwareMap.get(ColorSensor.class, "bottomColorR");
        tapeSensorR.enableLed(true);

        if (allianceColor != null) {
            initColorSensorSides();
        }

        Log.d(TAG, "initGlyphHardware: tape sensors initialized");

        tapeMap.put(outSensOutTape, false);
        tapeMap.put(inSensInTape, false);
        tapeMap.put(outSensInTape, false);

    }

    private void initColorSensorSides() {
        if (innerColor == null || outerColor == null) {
            if (allianceColor == AllianceColor.BLUE) {
                innerColor = tapeSensorR;
                outerColor = tapeSensorL;
                backCryptoboxAngle = 90;
                innerDirection = 270;
                outerDirection = 90;
            }
            else if (allianceColor == AllianceColor.RED) {
                innerColor = tapeSensorL;
                outerColor = tapeSensorR;
                backCryptoboxAngle = -90;
                innerDirection = 90;
                outerDirection = 270;
            }
        }
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
        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            Log.d(TAG, "pictographID: timed out, pictograph not found");
        } else {
            Log.d(TAG, "pictographID: pictograph found: " + vuMark.name());
        }
    }

    void place1stGlyph() throws InterruptedException {

        int optimalRangeCm = 25;

        if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addLine("VuMark Unknown");
        } if (vuMark == RelicRecoveryVuMark.LEFT) {
            telemetry.addLine("Glyph Left");
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 90, 0);
            Log.d(TAG, "place1stGlyph: started moving left");
            opMode.sleep(1400);
            mecanumDriveBase.mecanumDrive.stop();
            Log.d(TAG, "place1stGlyph: stopped moving left");
        } if (vuMark == RelicRecoveryVuMark.CENTER) {
            telemetry.addLine("Glyph Center");
        } if (vuMark == RelicRecoveryVuMark.RIGHT) {
            telemetry.addLine("Glyph Right");
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 270, 0);
            Log.d(TAG, "place1stGlyph: started moving right");
            opMode.sleep(1400);
            mecanumDriveBase.mecanumDrive.stop();
            Log.d(TAG, "place1stGlyph: stopped moving right");
        }
        telemetry.update();

        pickupHw.wristServo.setPosition(0);
        opMode.sleep(1000);
        pickupHw.wristServo.setPosition(0.5);
        mecanumDriveBase.turn(backCryptoboxAngle);
//        encoderDrive(0.3, 400, 1);
        rangeAdjustment(optimalRangeCm);

    }

    void jewelColor() throws InterruptedException {

        double timeLimit = 1.0;
        jewelArm.setPosition(0.4);
        colorSensor.enableLed(true);
        telemetry.addLine("Alliance Color: " + allianceColor.name());
        Log.d(TAG, "jewelColor: Alliance Color:" + allianceColor.name());
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
            Log.d(TAG, "jewelColor: sensed jewel is alliance color");
            telemetry.addLine("Moving right");
            mecanumDriveBase.turn(-10, 0.4);

            /*
            turnLeftWithoutAngle(0.5);
            while (opMode.opModeIsActive() && time.seconds() < timeLimit);
            mecanumDrive.stop();
            */
        }
        else {
            Log.d(TAG, "jewelColor: sensed jewel is not alliance color");
            telemetry.addLine("Moving left");
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

    boolean moveAwayFromColor() {
        return allianceColor == AllianceColor.BLUE && colorSensor.blue() > colorSensor.red()
                || allianceColor == AllianceColor.RED && colorSensor.red() > colorSensor.blue();
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
        Log.d(TAG, "driveToCryptobox: started moving forward off balancing stone");
        opMode.sleep(650);

        while (opMode.opModeIsActive() && (Math.abs(getAngleY()) > 2 || Math.abs(getAngleZ()) > 2)) {
            telemetry.addData("Angle Y", getAngleY());
            telemetry.addData("Angle Z", getAngleZ());
            telemetry.update();
        }
        Log.d(TAG, "driveToCryptobox: drove off balancing stone");
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "driveToCryptobox: stopped after driving off balancing stone");
        //encoderDrive(0.5, 400, 2);

        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.turn(90);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.turn(270);
        }
        Log.d(TAG, "driveToCryptobox: turned toward cryptobox");

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
        Log.d(TAG, "driveToCryptobox: started driving away from tape");

        while (opMode.opModeIsActive() && (runtime.seconds() < .75)) {
            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.5, 0, 0);
        }
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "driveToCryptobox: stopped driving away from tape");

        positionUsingTape();
        encoderDrive(0.3, 400, 1);
        Log.d(TAG, "driveToCryptobox: stopped in front of center column");
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

        int wallCm = 40;

       /*
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 90, 0);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 270, 0);
        }
        */

        ElapsedTime time = new ElapsedTime();
        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(.8, 0, 0);
        Log.d(TAG, "driveToCryptobox2: started moving forward off balancing stone");
        opMode.sleep(500);

        while (opMode.opModeIsActive() && (Math.abs(getAngleY()) > 2 || Math.abs(getAngleZ()) > 2)
                && time.seconds() < 1) {
            telemetry.addData("Angle Y", getAngleY());
            telemetry.addData("Angle Z", getAngleZ());
            telemetry.update();
        }
        Log.d(TAG, "driveToCryptobox2: drove off balancing stone");
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "driveToCryptobox2: stopped after driving off balancing stone");
        encoderDrive(0.5, 200, 2);

        mecanumDriveBase.turn(backCryptoboxAngle);
        Log.d(TAG, "driveToCryptobox2: turned toward cryptobox");

        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.6, 0, 0);
        Log.d(TAG, "driveToCryptobox2: started driving toward wall");
        while (opMode.opModeIsActive() && leftRange.getDistance(DistanceUnit.CM) > wallCm) {
            telemetry.addData("Left Distance", leftRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "driveToCryptobox2: stopped driving toward wall");

        mecanumDriveBase.turn(backCryptoboxAngle);

        //tapeFinder();
//        encoderColorDrive(0.6, 2500, 3);

//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.6, 0, 0);
//        opMode.sleep(1000);
//        mecanumDriveBase.mecanumDrive.stop();
        ElapsedTime runtime = new ElapsedTime();
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 270, 0);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 90, 0);
        }
        Log.d(TAG, "driveToCryptobox2: started driving away from tape");

        while (opMode.opModeIsActive() && (runtime.seconds() < .5)) {
            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.5, 0, 0);
        }
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "driveToCryptobox2: stopped driving away from tape");

        opMode.sleep(500);

        cryptoboxAngleCorrection();

        positionUsingTape3();
        Log.d(TAG, "driveToCryptobox2: stopped in front of center column");
//        encoderDrive(0.3, 400, 1);
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.turn(90);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.turn(270);
        }
        Log.d(TAG, "driveToCryptobox2: turned to realign with cryptobox");

//        encoderDrive(0.4, 300, 1);
//        Log.d(TAG, "driveToCryptobox2: drove forward to cryptobox");
////        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 0, 0);
////        opMode.sleep(400);
////        mecanumDriveBase.mecanumDrive.stop();
//
////        ElapsedTime time = new ElapsedTime();
////        time.reset();
////        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.8, 0, 0);
////
////        while (opMode.opModeIsActive() && time.seconds() < 1) {
////            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
////            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
////            telemetry.update();
////        }
////
////        mecanumDriveBase.mecanumDrive.stop();
////
////        if (allianceColor == AllianceColor.BLUE) {
////            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 270, 0);
////        } else if (allianceColor == AllianceColor.RED) {
////            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(1, 90, 0);
////        }
////
////        time.reset();
////        while (opMode.opModeIsActive() && time.seconds() < 1) {
////            //telemetry.addData("Left distance", leftRange.getDistance(DistanceUnit.CM));
////            //telemetry.addData("Right distance", rightRange.getDistance(DistanceUnit.CM));
////            telemetry.update();
////        }
////
////        mecanumDriveBase.mecanumDrive.stop();

        /*
        mecanumDrive.mecanumDrive_Polar(0.6, 90, 0);
        opMode.sleep(2000);
        mecanumDrive.stop();

        turn(90);
        */
    }

    void driveToSideCryptobox() throws InterruptedException {

        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(.8, innerDirection, 0);
        Log.d(TAG, "driveToSideCryptobox: started driving sideways off cryptobox");

        while (opMode.opModeIsActive() && (Math.abs(getAngleY()) > 2 || Math.abs(getAngleZ()) > 2)) {
            telemetry.addData("Angle Y", getAngleY());
            telemetry.addData("Angle Z", getAngleZ());
            telemetry.update();
        }
        Log.d(TAG, "driveToSideCryptobox: drove off balancing stone");
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "driveToSideCryptobox: stopped after driving off balancing stone");
        mecanumDriveBase.turn(180);

    }

    void cryptoboxAngleCorrection() throws InterruptedException {
        int strafeStartAngleOffset = 15;
        int strafeStartAngle = (int) (backCryptoboxAngle + Math.signum(-backCryptoboxAngle) * strafeStartAngleOffset);

        if (Math.abs(backCryptoboxAngle - getAngleX()) > strafeStartAngleOffset) {
            mecanumDriveBase.turn(strafeStartAngle);
            Log.d(TAG, "driveToCryptobox2: turn correction before positioning with tape");
        }
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
        ElapsedTime time = new ElapsedTime();
        int angleTolerance = 10;
        double resetTime = 1.2;

        boolean current;   // Used to make sure only the 2nd tape is read and not the 1st
        boolean previous;
        boolean far = false; // Tells whether 1st sensor reaches 2nd tapee before 2nd sensor reaches 1st tape

        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 90, 0);
            Log.d(TAG, "positionUsingTape: started moving toward triangle");
            time.reset();
            while (opMode.opModeIsActive() && tapeSensorL.blue() < BLUE_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Blue", tapeSensorL.blue());
                telemetry.addData("Right Tape Sensor: Blue", tapeSensorR.blue());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
//                if (Math.abs(backCryptoboxAngle - getAngleX()) > angleTolerance) {
                if (time.seconds() > resetTime) {
                    readjustTapePositioning();
                    Log.d(TAG, "positionUsingTape: readjusting angle");
                    time.reset();
                }
            }
            Log.d(TAG, "positionUsingTape: closer sensor found tape");
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
                    Log.d(TAG, "positionUsingTape: closer sensor found tape again");
                }
            }
            Log.d(TAG, "positionUsingTape: further sensor found tape");
            mecanumDriveBase.mecanumDrive.stop();
            Log.d(TAG, "positionUsingTape: stopped moving toward triangle");
            opMode.sleep(400);
            mecanumDriveBase.turn(90);
            if (far) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, -42, 0);
                Log.d(TAG, "positionUsingTape: started driving diagonally forward");
            }
            else {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, -222, 0);
                Log.d(TAG, "positionUsingTape: started driving diagonally backward");
            }
            while (opMode.opModeIsActive() && tapeSensorL.blue() < BLUE_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Blue", tapeSensorL.blue());
                telemetry.addData("Right Tape Sensor: Blue", tapeSensorR.blue());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
            }
            Log.d(TAG, "positionUsingTape: closer sensor found tape again");
        }
        else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, 270, 0);
            Log.d(TAG, "positionUsingTape: started moving toward triangle");
            while (opMode.opModeIsActive() && tapeSensorR.red() < RED_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Red", tapeSensorL.red());
                telemetry.addData("Right Tape Sensor: Red", tapeSensorR.red());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
//                if (Math.abs(backCryptoboxAngle - getAngleX()) > angleTolerance) {
                if (time.seconds() > resetTime) {
                    readjustTapePositioning();
                    Log.d(TAG, "positionUsingTape: readjusting angle");
                    time.reset();
                }
            }
            Log.d(TAG, "positionUsingTape: closer sensor found tape");
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
                    Log.d(TAG, "positionUsingTape: closer sensor found tape again");
                }
            }
            Log.d(TAG, "positionUsingTape: further sensor found tape");
            mecanumDriveBase.mecanumDrive.stop();
            Log.d(TAG, "positionUsingTape: stopped moving toward triangle");
            opMode.sleep(400);
            mecanumDriveBase.turn(270);
            if (far) {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 42, 0);
                Log.d(TAG, "positionUsingTape: started driving diagonally forward");
            }
            else {
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 222, 0);
                Log.d(TAG, "positionUsingTape: stopped moving toward triangle");
            }
            while (opMode.opModeIsActive() && tapeSensorR.red() < RED_THRESHOLD) {
                telemetry.addData("Left Tape Sensor: Red", tapeSensorL.red());
                telemetry.addData("Right Tape Sensor: Red", tapeSensorR.red());
                telemetry.addData("Boolean Far", far);
                telemetry.update(); //Tells the intensity of the blue color we are looking for
            }
            Log.d(TAG, "positionUsingTape: closer sensor found tape again");
        }
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "positionUsingTape: stopped moving diagonally");
        if (allianceColor == AllianceColor.BLUE) {
            mecanumDriveBase.turn(90);
        } else if (allianceColor == AllianceColor.RED) {
            mecanumDriveBase.turn(270);
        }
        Log.d(TAG, "positionUsingTape: turned toward cryptobox again");
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

    void positionUsingTape2() throws InterruptedException {
        int closeRangeCm = 10;
        int longRangeCm = 20;

        // Outer color sensor is between balancing stone and wall
        // First, find tape with outer sensor
        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, innerDirection, 0);
        Log.d(TAG, "positionUsingTape2: started driving toward tape");
        while (opMode.opModeIsActive() && !tapeMap.get(outSensInTape)) {
            tapeSearch();
            telemetry.update();
        }
        Log.d(TAG, "positionUsingTape2: outside sensor found tape");
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "positionUsingTape2: stopped after finding 1st tape");

        // Drive diagonally on outer tape until inner color sensor finds other side of triangle
        double forwardOutsideTapeDirection = Math.signum(backCryptoboxAngle) * 42;
        double backwardOutsideTapeDirection = forwardOutsideTapeDirection + 180;
        boolean innerTape = false; // For defensive programming if outer sensor is on inner tape
        if (tapeMap.get(inSensInTape)) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, forwardOutsideTapeDirection, 0);
            Log.d(TAG, "positionUsingTape2: moving forward along tape");
        } else {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, backwardOutsideTapeDirection, 0);
            Log.d(TAG, "positionUsingTape2: moving backward along tape");
        }

        while (opMode.opModeIsActive() && !allianceColorTapeFound(innerColor)) {
//            if (leftRange.getDistance(DistanceUnit.CM) < closeRangeCm) {
//                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, forwardOutsideTapeDirection, 0);
//            } else if (leftRange.getDistance(DistanceUnit.CM) > longRangeCm) {
//                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, backwardOutsideTapeDirection, 0);
//            }
            tapeSearch();
            telemetry.addData(allianceColor.name() + " Inner Color Value", tapeAllianceColorValue(innerColor));
            telemetry.update();
//            if (!allianceColorTapeFound(outerColor)) {
//                innerTape = true;
//                telemetry.log().add("Outer color sensor no longer on tape");
//                Log.d(TAG, "positionUsingTape2: Outer color sensor no longer on tape");
//                break;
//            }
        }
        Log.d(TAG, "positionUsingTape2: found tape with triangle");
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "positionUsingTape2: stopped aligned with triangle");

//        if (innerTape) { // Only runs if outer color sensor drove off tape
//            // Drives outward until outer sensor finds outer tape
//            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, outerDirection, 0);
//            while (opMode.opModeIsActive() && !allianceColorTapeFound(outerColor)) {
//                telemetry.addData( allianceColor.name() + " Outer Color Value", tapeAllianceColorValue(outerColor));
//                telemetry.update();
//            }
//            mecanumDriveBase.mecanumDrive.stop();
//
//            // Drives diagonally toward cryptobox along inner tape until inner sensor finds other side of triangle
//            double forwardInsideTapeDirection = Math.signum(-backCryptoboxAngle) * 42;
//            double backwardInsideTapeDirection = forwardInsideTapeDirection + 180;
//            while (opMode.opModeIsActive() && !allianceColorTapeFound(innerColor)) {
//                if (leftRange.getDistance(DistanceUnit.CM) < closeRangeCm) {
//                    mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, forwardInsideTapeDirection, 0);
//                } else if (leftRange.getDistance(DistanceUnit.CM) > longRangeCm) {
//                    mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, backwardInsideTapeDirection, 0);
//                }
//                telemetry.addData(allianceColor.name() + " Inner Color Value", tapeAllianceColorValue(innerColor));
//                telemetry.update();
//            }
//            mecanumDriveBase.mecanumDrive.stop();
//        }
    } // Starts after robot strafed off back balancing stone

    void positionUsingTape3() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();
        int angleTolerance = 10;
        double resetTime = 2;
        boolean outerSensorSkipped = false;

        double forwardInsideTapeDirection = Math.signum(-backCryptoboxAngle) * 42;
        double backwardInsideTapeDirection = forwardInsideTapeDirection + 180;

        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, outerDirection, 0);
        Log.d(TAG, "positionUsingTape3: started moving toward triangle");
        time.reset();
        while (opMode.opModeIsActive() && !tapeMap.get(inSensInTape)) {
            tapeSearch();
            telemetry.update(); //Tells the intensity of the blue color we are looking for
//                if (Math.abs(backCryptoboxAngle - getAngleX()) > angleTolerance) {
            if (tapeMap.get(inSensInTape) && !tapeMap.get(outSensInTape)) { // Inner sensor finding tape before outer indicates problem
                Log.d(TAG, "positionUsingTape3: Error: wrong tape order");
                mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.7, innerDirection, 0);
                outerSensorSkipped = true;
            }
            if (time.seconds() > resetTime) {
                readjustTapePositioning();
                Log.d(TAG, "positionUsingTape3: readjusting angle");
                time.reset();
            }
        }
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "positionUsingTape3: stopped moving toward triangle");
        opMode.sleep(400);
        mecanumDriveBase.turn(backCryptoboxAngle);
        if (tapeMap.get(outSensOutTape)) {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, forwardInsideTapeDirection, 0);
            Log.d(TAG, "positionUsingTape3: started driving diagonally forward");
        }
        else {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, backwardInsideTapeDirection, 0);
            Log.d(TAG, "positionUsingTape3: started driving diagonally backward");
        }
        while (opMode.opModeIsActive() && !allianceColorTapeFound(outerColor)) {
            tapeSearch();
            telemetry.update(); //Tells the intensity of the blue color we are looking for
        }
        Log.d(TAG, "positionUsingTape3: outer sensor found tape again");
        mecanumDriveBase.mecanumDrive.stop();
        Log.d(TAG, "positionUsingTape3: stopped moving diagonally");
        mecanumDriveBase.turn(backCryptoboxAngle);
        Log.d(TAG, "positionUsingTape3: turned toward cryptobox again");
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

    public void rangeAdjustment(int optimalRangeCm){
        double producedRangeCm = 0.5*(leftRange.getDistance(DistanceUnit.CM)+rightRange.getDistance(DistanceUnit.CM));


        if (producedRangeCm<optimalRangeCm){
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 180, 0);
            while(producedRangeCm<optimalRangeCm){
                producedRangeCm = 0.5*(leftRange.getDistance(DistanceUnit.CM)+rightRange.getDistance(DistanceUnit.CM));
                telemetry.addData ("Distance Left to Move", optimalRangeCm-producedRangeCm);
                telemetry.update();
            }
        } else {
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 0, 0);
            while(producedRangeCm>optimalRangeCm){
                producedRangeCm = 0.5*(leftRange.getDistance(DistanceUnit.CM)+rightRange.getDistance(DistanceUnit.CM));
                telemetry.addData ("Distance Left to Move", optimalRangeCm-producedRangeCm);
                telemetry.update();
        }
        mecanumDriveBase.mecanumDrive.stop(); }
    }

    private void tapeSearch() {
        // Set variables to previous state of each sensor
        boolean prevInside = insideColorSensed;
        boolean prevOutside = outsideColorSensed;
        insideColorSensed = allianceColorTapeFound(innerColor);
        outsideColorSensed = allianceColorTapeFound(outerColor);
        // Set values in hashmap by comparing current and previous states of sensors
        if (outsideColorSensed && !prevOutside) {
            if (!tapeMap.get(outSensInTape)) {
                tapeMap.put(outSensInTape, true);
                Log.d(TAG, "tapeSearch: " + outSensInTape + " found");
            } else {
                tapeMap.put(outSensOutTape, true);
                Log.d(TAG, "tapeSearch: " + outSensOutTape + " found");
            }
        }
        if (insideColorSensed && !prevInside) {
            tapeMap.put(inSensInTape, true);
            Log.d(TAG, "tapeSearch: " + inSensInTape + " found");
        }
        // Add telemetry with color value of each sensor
        telemetry.addData("Inner Sensor " + allianceColor.name(), tapeAllianceColorValue(innerColor));
        telemetry.addData("Outer Sensor " + allianceColor.name(), tapeAllianceColorValue(outerColor));
        telemetry.addData(outSensInTape, tapeMap.get(outSensInTape));
        telemetry.addData(outSensOutTape, tapeMap.get(outSensOutTape));
        telemetry.addData(inSensInTape, tapeMap.get(inSensInTape));
        // Add telemetry showing values in hashmap
    } // To be used inside while loop, keeps track of tape states and displays, update telemetry after calling to display

    void readjustTapePositioning() throws InterruptedException {
        int wallCm = 38;

        mecanumDriveBase.turn(backCryptoboxAngle, this, "tapeSearch");

        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.4, 0, 0);
        while (opMode.opModeIsActive() && leftRange.getDistance(DistanceUnit.CM) > wallCm) {
            tapeSearch();
            telemetry.addData("Wall Distance", leftRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        mecanumDriveBase.mecanumDrive.stop();
    }

    boolean allianceColorTapeFound(ColorSensor colorSensor) {
        int threshold = Integer.MAX_VALUE; // returns false if alliance color is not initialized
        if (allianceColor == AllianceColor.BLUE) {
            threshold = BLUE_THRESHOLD;
        } else if (allianceColor == AllianceColor.RED) {
            threshold = RED_THRESHOLD;
        }
        return tapeAllianceColorValue(colorSensor) > threshold;
    }

    private double tapeAllianceColorValue(ColorSensor colorSensor) {
        if (allianceColor == AllianceColor.BLUE) {
            return colorSensor.blue();
        } else if (allianceColor == AllianceColor.RED) {
            return colorSensor.red();
        } else {
            telemetry.log().add("Alliance color not initialized");
            Log.d(TAG, "tapeAllianceColorValue: Alliance color not initialized");
            return -1;
        }
    }

    void moveFWBW() {
        ElapsedTime     runtime = new ElapsedTime();

        Log.d(TAG, "moveFWBW: moving backward slightly");
        encoderDrive(0.5, -200, 1);
        opMode.sleep(500);
        Log.d(TAG, "moveFWBW: moving backward more");
        encoderDrive(0.5, -250, 1);
        Log.d(TAG, "moveFWBW: moving forward");
        encoderDrive(0.5, 450, 1);
        Log.d(TAG, "moveFWBW: moving backward again");
        encoderDrive(0.5, -200, 1);

//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 180, 0);
//        Log.d(TAG, "moveFWBW: moving backward");
//        while (opMode.opModeIsActive() && (runtime.seconds() < 0.2)) {
//            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.5, 0, 0);
//        }
//        mecanumDriveBase.mecanumDrive.stop();
//        opMode.sleep(500);
//
//        runtime.reset();
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 180, 0);
//        Log.d(TAG, "moveFWBW: moving backward");
//        while (opMode.opModeIsActive() && (runtime.seconds() < 0.2)) {
//            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(-0.5, 0, 0);
//        }
//        mecanumDriveBase.mecanumDrive.stop();
//
//        runtime.reset();
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 0, 0);
//        Log.d(TAG, "moveFWBW: moving forward");
//        while (opMode.opModeIsActive() && (runtime.seconds() < 0.4)) {
//            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(0.5, 0, 0);
//        }
//        mecanumDriveBase.mecanumDrive.stop();
//
//        runtime.reset();
//        mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 180, 0);
//        Log.d(TAG, "moveFWBW: moving backward");
//        while (opMode.opModeIsActive() && (runtime.seconds() < 0.4)) {
//            //mecanumMotors.mecanumDrive.mecanumDrive_XPolar(-0.5, 0, 0);
//        }
//        mecanumDriveBase.mecanumDrive.stop();
//        Log.d(TAG, "moveFWBW: stopped");
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
        pickupHw.leftServo.setPosition(0);
        pickupHw.rightServo.setPosition(1.0);
        Log.d(TAG, "ejectGlyph: servos moving outward");
        while (opMode.opModeIsActive() && time.seconds() < 2);
        pickupHw.leftServo.setPosition(0.5);
        pickupHw.rightServo.setPosition(0.5);
        Log.d(TAG, "ejectGlyph: servos stopped");
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
            newRightTarget = motorR.getCurrentPosition() + (int)(encoderCounts);
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
            Log.d(TAG, "encoderColorDrive: started moving with encoders while searching for color");

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
                    Log.d(TAG, "encoderColorDrive: closer sensor found tape");
                    telemetry.log().add("Left found pos: " + motorL.getCurrentPosition());
                    telemetry.log().add("Right found pos: " + motorR.getCurrentPosition());
                }

                if (firstTapeFound
                        && ((motorL.getCurrentPosition() - startPosL) > minDist || (motorR.getCurrentPosition() - startPosR) > minDist)) {
                    mecanumDriveBase.mecanumDrive.stop();
                    Log.d(TAG, "encoderColorDrive: stopped after finding color and passing minimum distance");
                    motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    opMode.sleep(200);
                    return;
                }
                // Display it for the driver.
                telemetry.addData("1st Tape Found", firstTapeFound);
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorL.getCurrentPosition(),
                        motorR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            mecanumDriveBase.mecanumDrive.stop();
            Log.d(TAG, "encoderColorDrive: stopped robot");

            // Turn off RUN_TO_POSITION
            motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            opMode.sleep(200);

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
            newRightTarget = motorR.getCurrentPosition() + (int)(encoderCounts);
            motorL.setTargetPosition(newLeftTarget);
            motorR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            mecanumDriveBase.setBrakeModeEnabled(true);
            mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(speed, -Math.signum(encoderCounts) * 90 + 90, 0);
            Log.d(TAG, "encoderDrive: started moving robot");

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
            Log.d(TAG, "encoderDrive: stopped robot");

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

    @Override
    public void accept(String method) {
        if (method.equals("tapeSearch")) {
            tapeSearch();
        }
    }
}
