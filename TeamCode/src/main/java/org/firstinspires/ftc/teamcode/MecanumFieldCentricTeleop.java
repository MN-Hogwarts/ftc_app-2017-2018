package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;

import ftclib.FtcDcMotor;
import ftclib.FtcServo;
import hallib.HalDashboard;
import swlib.SWGamePad;
import swlib.SWIMUGyro;
import swlib.SwDriveBase;
import trclib.TrcGyro;
import trclib.TrcRobot;
import trclib.TrcServo;
import trclib.TrcTaskMgr;

/**
 * Created by spmeg on 4/15/2017.
 */
@TeleOp(name = "MecanumFieldCentricTeleop", group = "teleop")
@Disabled
public class MecanumFieldCentricTeleop extends OpMode implements SWGamePad.ButtonHandler, Runnable{
    /*
    private final double LOADED_POSITION_TRIGGER = 0.557;
    private final double SHOOT_POSITION_TRIGGER = 0.428;
    private final double START_POSITION_ANGULAR = 0.3;
    private final double START_POSITION_TURRET = 0.50;
    private final double CLOCKWISE_POSITION_TURRET = 0.40;
    private final double COUNTERCLOCKWISE_POSITION_TURRET = 0.60;
    private final double SHOOTER_POWER = 1.0;
    private final double REVERSE_SHOOTER_POWER = -0.5;
    private final double SERVO_INCREMENT = 0.0025;

    private Servo triggerServo, angularServo, turretServo;
    */

    private boolean OP_MODE_IS_ACTIVE = true;
    /*
    private boolean SHOOTER_MOTORS_ACTIVE = false;
    private boolean SHOOTER_MOTORS_REVERSE = false;
    private boolean SHOOTER_MOTORS_IS_READY = false;

    private DcMotor leftShooter, rightShooter;

    boolean               inputPin;             // Input State
    DigitalChannel digIn;                // Device Object
    */

    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;
    private SwDriveBase driveBase = null;
    private SWIMUGyro gyro = null;
    private SWGamePad gamepad;
    private boolean fixedOnTarget = false;
    private HalDashboard dashboard = null;
    private final int LABEL_WIDTH = 200;
    private boolean setYInverted = true;
    private boolean setXInverted = false;
    private TrcTaskMgr taskMgr = new TrcTaskMgr();
    private HashMap<Integer, ElapsedTime> toggleTimeTracker = new HashMap<>();
    private TrcServo jewelServo = null;
    private Servo wristServo;
    private Servo leftPickupServo, rightPickupServo;
    private DigitalChannel touchSensor ;
    static double wristServoValue = 0.04;
    private static double gyroKp = 0.3;
    private static double gyroScale = 0.5;

    @Override
    public void init_loop() {
        super.init_loop();
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.TELEOP_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
    }

    @Override
    public void start() {
        super.start();
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, TrcRobot.RunMode.TELEOP_MODE);
    }

    @Override
    public void stop() {
        super.stop();
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, TrcRobot.RunMode.TELEOP_MODE);
        OP_MODE_IS_ACTIVE = false;
    }



    @Override
    public void init() {
        dashboard = HalDashboard.createInstance(this.telemetry);
        jewelServo = new FtcServo(this.hardwareMap, "jewelArm");
        jewelServo.setPosition(0.9);

        leftPickupServo = this.hardwareMap.get(Servo.class, "leftPickup");
        rightPickupServo = this.hardwareMap.get(Servo.class, "rightPickup");
        wristServo = this.hardwareMap.get(Servo.class, "wristServo1");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");

        gyro = new SWIMUGyro(hardwareMap, "imu", null);
        gyro.calibrate();

        leftFrontMotor = new FtcDcMotor(this.hardwareMap, "leftFront", null, null);
        leftRearMotor = new FtcDcMotor(this.hardwareMap, "leftRear", null, null);
        rightFrontMotor = new FtcDcMotor(this.hardwareMap, "rightFront", null, null);
        rightRearMotor = new FtcDcMotor(this.hardwareMap, "rightRear", null, null);

        leftFrontMotor.setInverted(true);
        leftRearMotor.setInverted(true);

        leftFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBase = new SwDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);
        /*driveBase = new TrcDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);*/

        gyro.setEnabled(true);

        gamepad = new SWGamePad("driver gamepad", gamepad1, 0.05F);
        gamepad.enableDebug(true);

        //driveBase.enableGyroAssist(gyroScale, gyroKp);
        /*
        triggerServo = hardwareMap.servo.get("triggerServo");
        angularServo = hardwareMap.servo.get("pixyyaxis");
        turretServo = hardwareMap.servo.get("pixyyrotation");

        leftShooter = hardwareMap.dcMotor.get("shooterLeft");
        rightShooter = hardwareMap.dcMotor.get("shooterRight");

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        digIn  = hardwareMap.get(DigitalChannel.class, "digin");     //  Use generic form of device mapping
        digIn.setMode(DigitalChannelController.Mode.INPUT);          // Set the direction of each channel

        triggerServo.setPosition(LOADED_POSITION_TRIGGER);
        angularServo.setPosition(START_POSITION_ANGULAR);
        turretServo.setPosition(START_POSITION_TURRET);
        */

        new Thread(this).start();
    }

    @Override
    public void loop() {
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.TELEOP_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.AUTO_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.AUTO_MODE);

        gamepad.setYInverted(setYInverted);
        double rotation = gamepad.getRightStickX()*-1;
        double magnitude = Range.clip(gamepad.getLeftStickMagnitude(), 0, 1);
        double direction = gamepad.getLeftStickDirectionDegrees(true);

        if(gamepad.getLeftStickX() == 0 && gamepad.getLeftStickY() == 0)
            magnitude = 0;

        driveBase.mecanumDrive_XPolarFieldCentric(magnitude, direction, rotation);
        //driveBase.mecanumDrive_XPolar(magnitude, direction, rotation);

        if(gamepad1.left_bumper){
            //gyroScale += 0.05;
            jewelServo.setPosition(0.9);
        } else if(gamepad1.left_trigger > 0.3){
            //gyroScale -= 0.05;
        } else {
            ////driveBase.enableGyroAssist(gyroScale, gyroKp);
        }

        if(gamepad1.right_bumper){
            //gyroKp += 0.05;
        } else if(gamepad1.right_trigger > 0.3){
            //gyroKp -= 0.05;
        } else {
            //driveBase.enableGyroAssist(gyroScale, gyroKp);
        }

        if (gamepad2.b) {
            leftPickupServo.setPosition(-1.0);
            rightPickupServo.setPosition(1.0);
        } //If touch sensor is pressed, stop wheels. If 'A' is pressed, run wheels. If neither is pressed, stop wheels
        else if (!touchSensor.getState()) {
            leftPickupServo.setPosition(0.53);
            rightPickupServo.setPosition(0.5);
        } //Turn inward
        else if (gamepad2.a) {
            leftPickupServo.setPosition(1.0);
            rightPickupServo.setPosition(-1.0);
        } //Stop wheels
        else {
            leftPickupServo.setPosition(0.53);
            rightPickupServo.setPosition(0.5);
        }

        if(gamepad2.a){
            //wristServo1.setPosition(0.5);
            //wristServoValue = wristServoValue - 0.05;
        } else if (gamepad2.y){
            wristServo.setPosition(1);
            //wristServoValue = wristServoValue + 0.05;
        } else if (gamepad2.x) {
            //wristServoValue = Range.clip(wristServoValue, -1.0, 1.0);
            //wristServo1.setPosition(wristServoValue);
            wristServo.setPosition(0);
        } else {
            wristServo.setPosition(0.5);
        }

        /*
        if(SHOOTER_MOTORS_REVERSE){
            leftShooter.setPower(REVERSE_SHOOTER_POWER);
            rightShooter.setPower(REVERSE_SHOOTER_POWER);
        } else if (SHOOTER_MOTORS_ACTIVE){
            leftShooter.setPower(SHOOTER_POWER);
            rightShooter.setPower(SHOOTER_POWER);
        } else {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }

        dashboard.displayPrintf(1, LABEL_WIDTH, "digital input: ", "%b", inputPin);
        dashboard.displayPrintf(2, LABEL_WIDTH, "servo power: ", "%1.3f", triggerServo.getPosition());
        dashboard.displayPrintf(4, LABEL_WIDTH, "rotation: ", "%.2f", rotation);
        */
        dashboard.displayPrintf(2, LABEL_WIDTH, "imu z: ", "%.2f", gyro.getZRotationRate().value);
        dashboard.displayPrintf(3, LABEL_WIDTH, "gamepad left stick mag: ", "%.2f", magnitude);
        dashboard.displayPrintf(4, LABEL_WIDTH, "wristValue: ", "%1.2f", wristServo.getPosition());
        dashboard.displayPrintf(5, LABEL_WIDTH, "leftPickupServo: ", "%1.2f", leftPickupServo.getPosition());
        dashboard.displayPrintf(6, LABEL_WIDTH, "rightPickupServo: ", "%1.2f", rightPickupServo.getPosition());
        dashboard.displayPrintf(7, LABEL_WIDTH, "gamepad left stick direction true: ", "%.2f", gamepad.getLeftStickDirectionDegrees(true));
        dashboard.displayPrintf(8, LABEL_WIDTH, "gamepad left stick direction false: ", "%.2f", gamepad.getLeftStickDirectionDegrees(false));
        dashboard.displayPrintf(9, LABEL_WIDTH, "gamepad right stick x: ", "%1.2f", gamepad.getRightStickX());
        dashboard.displayPrintf(10, LABEL_WIDTH, "gamepad right stick y: ", "%1.2f", gamepad.getRightStickY());
        dashboard.displayPrintf(11, LABEL_WIDTH, "y inverted: ", "%b", setYInverted);
        dashboard.displayPrintf(12, LABEL_WIDTH, "x inverted: ", "%b", setXInverted);
        dashboard.displayPrintf(13, LABEL_WIDTH, "fixedOnTarget: ", "%b", fixedOnTarget);
        dashboard.displayPrintf(14, LABEL_WIDTH, "imu x: ", "%.2f", gyro.getRawXData(TrcGyro.DataType.HEADING).value);
        dashboard.displayPrintf(15, LABEL_WIDTH, "imu y: ", "%.2f", gyro.getRawYData(TrcGyro.DataType.HEADING).value);
        dashboard.displayPrintf(16, LABEL_WIDTH, "imu z: ", "%.2f", gyro.getRawZData(TrcGyro.DataType.HEADING).value);
        dashboard.displayPrintf(17, LABEL_WIDTH, "gyro scale: ", "%.2f", gyroScale);
        dashboard.displayPrintf(18, LABEL_WIDTH, "gyro kp: ", "%.2f", gyroKp);
        jewelServo.setPosition(0.9);
    }

    private ElapsedTime getButtonElapsedTime(int button){
        if(toggleTimeTracker.containsKey(button))
            return toggleTimeTracker.get(button);
        else {
            toggleTimeTracker.put(button, new ElapsedTime());
            return toggleTimeTracker.get(button);
        }
    }

    @Override
    public void gamepadButtonEvent(SWGamePad gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case SWGamePad.GAMEPAD_A:
                    //driveBase.enableGyroAssist(0.00001, 0.05);
                    break;

                case SWGamePad.GAMEPAD_Y:
                    if(pressed && getButtonElapsedTime(button).seconds() > 1){
                        fixedOnTarget = !fixedOnTarget;
                        getButtonElapsedTime(button).reset();
                    }
                    break;

                case SWGamePad.GAMEPAD_X:
                    break;

                case SWGamePad.GAMEPAD_B:
                    //driveBase.disableGyroAssist();
                    break;

                case SWGamePad.GAMEPAD_RBUMPER:
                    break;

                /*
                case SWGamePad.GAMEPAD_DPAD_DOWN:
                    break;

                case SWGamePad.GAMEPAD_DPAD_UP:
                    if(pressed && getButtonElapsedTime(button).seconds() > 1){
                        if(!checkToggleTime(button))
                            break;

                        setYInverted = !setYInverted;
                        getButtonElapsedTime(button).reset();
                    }
                    break;
                 */

                case SWGamePad.GAMEPAD_DPAD_LEFT:
                    break;

                case SWGamePad.GAMEPAD_DPAD_RIGHT:
                    if(pressed && getButtonElapsedTime(button).seconds() > 1){
                        if(!checkToggleTime(button))
                            break;

                        setXInverted = !setXInverted;
                        getButtonElapsedTime(button).reset();
                    }
                    break;
            }
        }
    }

    private boolean checkToggleTime(int button){
        return (getButtonElapsedTime(button).seconds() > 0.75);
    }

    //THE THREAD THAT WILL RUN THE TRIGGER
    @Override
    public void run() {
        /*
        while (OP_MODE_IS_ACTIVE){
            inputPin = digIn.getState();
            if(!gamepad1.left_bumper) {
                SHOOTER_MOTORS_REVERSE = false;
                if (inputPin && gamepad1.right_bumper) {

                    SHOOTER_MOTORS_ACTIVE = true;

                    //if the shooter motors are not ready, SPEED THEM UP
                    if (!SHOOTER_MOTORS_IS_READY) {
                        for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 250; ) {
                            SHOOTER_MOTORS_ACTIVE = true;
                        }

                        //notify opmode that shooter motors are ready
                        SHOOTER_MOTORS_IS_READY = true;
                    }

                    for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 450; ) {
                        triggerServo.setPosition(SHOOT_POSITION_TRIGGER);
                    }
                } else if (inputPin) {

                    SHOOTER_MOTORS_ACTIVE = true;

                    //if the shooter motors are not ready, SPEED THEM UP
                    if (!SHOOTER_MOTORS_IS_READY) {
                        for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 250; ) {
                            SHOOTER_MOTORS_ACTIVE = true;
                        }
                        //notify opmode that shooter motors are ready
                        SHOOTER_MOTORS_IS_READY = true;
                    }

                    for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 250; ) {
                        triggerServo.setPosition(LOADED_POSITION_TRIGGER);
                    }
                } else {
                    SHOOTER_MOTORS_ACTIVE = false;
                    SHOOTER_MOTORS_IS_READY = false;

                    for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 250; ) {
                        triggerServo.setPosition(LOADED_POSITION_TRIGGER);
                    }
                }
            } else if (gamepad1.b) {
                SHOOTER_MOTORS_ACTIVE = false;
                SHOOTER_MOTORS_IS_READY = false;
                SHOOTER_MOTORS_REVERSE = true;
            }
        }
        */
    }
}
