package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import ftclib.FtcDcMotor;
import ftclib.FtcServo;
import hallib.HalDashboard;
import swlib.SWGamePad;
import swlib.SWIMUGyro;
import swlib.SwDriveBase;
import trclib.TrcGyro;
import trclib.TrcRobot;
import trclib.TrcSensor;
import trclib.TrcServo;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * Created by spmeg on 4/15/2017.
 */
@TeleOp(name = "MecanumRobotCentricTeleop", group = "teleop")
public class MecanumRobotCentricTeleop extends OpMode{

    private static boolean OP_MODE_IS_ACTIVE = true;

    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;
    private FtcDcMotor armMotor;
    private double armMotorSpeedLimiter = 0.3;
    private SwDriveBase driveBase = null;
    private SWIMUGyro gyro = null;
    private SWGamePad gamepad;
    //private HalDashboard dashboard = null;
    private boolean setYInverted = true;
    private TrcServo jewelServo = null;
    private Servo wristServo;
    private Servo leftPickupServo, rightPickupServo;
    private Servo relicServo;
    private DigitalChannel touchSensor ;
    private boolean turtleMode = false;
    private double magnitude = 0;
    //private boolean adjustGyroScale = true;
    private double relicServPos = 0.7;

    private static double gyroKp = 0;
    private static double gyroScale = 1.;
    private static List<TrcSensor.SensorData<Double>> maRotationRate = new LinkedList<>();
    private static int maxListSize = 10;
    private static double rotationRate = 0;

    @Override
    public void stop() {
        super.stop();
        OP_MODE_IS_ACTIVE = false;
    }



    @Override
    public void init() {
        //dashboard = HalDashboard.createInstance(this.telemetry);
        jewelServo = new FtcServo(this.hardwareMap, "jewelArm");
        jewelServo.setPosition(0.9);

        leftPickupServo = this.hardwareMap.get(Servo.class, "leftPickup");
        rightPickupServo = this.hardwareMap.get(Servo.class, "rightPickup");
        wristServo = this.hardwareMap.get(Servo.class, "wristServo");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        relicServo = hardwareMap.get(Servo.class, "relicServo");
        //pickupHw.init(hardwareMap);

        gyro = new SWIMUGyro(hardwareMap, "imu", null);
        gyro.calibrate();

        leftFrontMotor = new FtcDcMotor(this.hardwareMap, "leftFront", null, null);
        leftRearMotor = new FtcDcMotor(this.hardwareMap, "leftRear", null, null);
        rightFrontMotor = new FtcDcMotor(this.hardwareMap, "rightFront", null, null);
        rightRearMotor = new FtcDcMotor(this.hardwareMap, "rightRear", null, null);
        armMotor = new FtcDcMotor(this.hardwareMap, "armMotor", null, null);

        armMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setInverted(true);

        leftFrontMotor.setInverted(true);
        leftRearMotor.setInverted(true);

        leftFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBase = new SwDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);
        /*driveBase = new TrcDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMo tor, rightRearMotor);*/

        gyro.setEnabled(true);

        gamepad = new SWGamePad("driver gamepad", gamepad1, 0.05F);
        gamepad.enableDebug(true);

        //driveBase.enableGyroAssist(gyroScale, gyroKp);

        new Thread(new Runnable() {

            @Override
            public void run() {
                while (OP_MODE_IS_ACTIVE){
                    if(maRotationRate.size() > maxListSize){
                        maRotationRate.remove(0);
                        maRotationRate.add(gyro.getZRotationRate());
                    }

                    double sum = 0;
                    for(TrcSensor.SensorData<Double> rateValue: maRotationRate){
                        sum = sum + rateValue.value;
                    }

                    rotationRate = sum/maxListSize;
                }
            }

        });

        new Thread(new Runnable() {
            @Override
            public void run() {
                while (OP_MODE_IS_ACTIVE){
                    gamepad.setYInverted(setYInverted);
                    double rotation = gamepad.getRightStickX()*-1;
                    magnitude = Range.clip(gamepad.getLeftStickMagnitude(), 0, 1);
                    if(turtleMode)
                        magnitude = magnitude/2;

                    double direction = gamepad.getLeftStickDirectionDegrees(true);

                    if(gamepad.getLeftStickX() == 0 && gamepad.getLeftStickY() == 0)
                        magnitude = 0;

                    double addRotation = gyroScale*rotationRate;

                    rotation += TrcUtil.clipRange(gyroKp*(rotation - addRotation));

                    //driveBase.mecanumDrive_XPolarFieldCentric(magnitude, direction, rotation);
                    driveBase.mecanumDrive_XPolar(magnitude, direction, rotation);
                    jewelServo.setPosition(0.9);

                    if(gamepad1.a){
                        turtleMode = true;
                    } else if(gamepad1.b){
                        turtleMode = false;
                    }
                }
            }
        }).start();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while (OP_MODE_IS_ACTIVE){
                    if(gamepad2.dpad_up){
                        armMotorSpeedLimiter = armMotorSpeedLimiter + 0.01;
                    } else if(gamepad2.dpad_down){
                        armMotorSpeedLimiter = armMotorSpeedLimiter - 0.01;
                    }

                    armMotorSpeedLimiter = Range.clip(armMotorSpeedLimiter, 0, 1);

                    armMotor.setPower(gamepad2.left_stick_y*armMotorSpeedLimiter);

                    if (gamepad1.dpad_right){
                        //relicServo.setPosition(0.3);
                        relicServPos = 0.3;
                    } else if (gamepad1.dpad_left){
                        //relicServo.setPosition(0.7);
                        relicServPos = 0.7;
                    }

                    relicServo.setPosition(relicServPos);
                }
            }
        }).start();

        new Thread(new Runnable() {
            @Override
            public void run() {
                while (OP_MODE_IS_ACTIVE){
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
                    else if (gamepad2.left_bumper) {
                        leftPickupServo.setPosition(1.0);
                        rightPickupServo.setPosition(0.5);
                    }
                    else if (gamepad2.right_bumper) {
                        rightPickupServo.setPosition(-1.0);
                        leftPickupServo.setPosition(0.53);
                    }
                    else {
                        leftPickupServo.setPosition(0.53);
                        rightPickupServo.setPosition(0.5);
                    }

                    if (gamepad2.y){
                        wristServo.setPosition(1);
                        //wristServoValue = wristServoValue + 0.05;
                    } else if (gamepad2.x) {
                        //wristServoValue = Range.clip(wristServoValue, -1.0, 1.0);
                        //wristServo.setPosition(wristServoValue);
                        wristServo.setPosition(0);
                    } else {
                        wristServo.setPosition(0.5);
                    }
                }
            }
        }).start();
    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper)
            maxListSize++;
        else if(gamepad1.left_trigger > 0.3)
            maxListSize--;

        if(gamepad1.right_bumper)
            gyroKp = gyroKp + 0.05;
        else if(gamepad1.right_trigger > 0.3)
            gyroKp = gyroKp - 0.05;

        if(gamepad1.dpad_up)
            gyroScale = gyroScale + 0.05;
        else if(gamepad1.dpad_down)
            gyroScale = gyroScale - 0.05;

        telemetry.addData("magnitude", magnitude);
        telemetry.addData("Arm Speed Limiter", armMotorSpeedLimiter);
        telemetry.addData("turtle mode", turtleMode);
        telemetry.addData("z rotation rate", gyro.getZRotationRate().value);
        telemetry.addData("ma z rotation rate", rotationRate);
        telemetry.addData("maxListSize", maxListSize);
        telemetry.addData("gyroKp", gyroKp);
        telemetry.addData("gyroScale", gyroScale);
        telemetry.update();
    }
}
