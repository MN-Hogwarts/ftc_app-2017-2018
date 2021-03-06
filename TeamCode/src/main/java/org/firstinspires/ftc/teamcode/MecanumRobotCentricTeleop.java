package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import ftclib.FtcDcMotor;
import ftclib.FtcServo;
import swlib.SWGamePad;
import swlib.SwDriveBase;
import trclib.TrcSensor;
import trclib.TrcServo;

/**
 * Created by spmeg on 4/15/2017.
 */
@TeleOp(name = "MecanumRobotCentricTeleop", group = "teleop")
public class MecanumRobotCentricTeleop extends OpMode{

    private volatile static boolean OP_MODE_IS_ACTIVE = false;

    private int WristOrientation = 1;
    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;
    private FtcDcMotor armMotor;
    private double armMotorSpeedLimiter = 0.53;
    private SwDriveBase driveBase = null;
    //private SWIMUGyro gyro = null;
    private SWGamePad gamepad;
    //private HalDashboard dashboard = null;
    private boolean setYInverted = true;
    private TrcServo jewelServo = null;
    private CRServo wristServo1, wristServo2;
    private Servo rightHinge, leftHinge;
    private CRServo leftPickupServo, rightPickupServo;
    private CRServo leftPickupServo2, rightPickupServo2;
    private Servo relicServo;
    private DigitalChannel touchSensor ;
    private boolean turtleMode = false;
    private double magnitude = 0;
    //private boolean adjustGyroScale = true;
    private double relicServPos = 0.8;

    //private static double gyroKp = 0;
    //private static double gyroScale = 1.;
    private static List<TrcSensor.SensorData<Double>> maRotationRate = new LinkedList<>();
    private static int maxListSize = 10;
    private static double rotationRate = 0;

    static final double INCREMENT   = 0.3;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.8;     // Maximum rotational power
    static final double MIN_POS     =  -0.8;     // Minimum rotational power
    static final double MAX_FINGER_POS = 0.8;
    static final double MIN_FINGER_POS = -0.8;
    static final double LEFT_STOP_POS   = 0.025;
    static final double RIGHT_STOP_POS   = 0;

    private boolean hingeUpR = true;
    private boolean hingeUpL = true;
    private boolean pressedTriggerR = false;
    private boolean prevPressedTriggerR = false;
    private boolean pressedTriggerL = false;
    private boolean prevPressedTriggerL = false;

    double  position = 0.5;
    double rightHingePos = 0.2;
    double leftHingePos = 0.9;

    private ExecutorService executorService;
    private boolean initFinished = false;

    Runnable driveRunnable, armRunnable, servoRunnable;

    @Override
    public void stop() {
        super.stop();
        OP_MODE_IS_ACTIVE = false;
    }

    private void sleep(int millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    @Override
    public void init() {
        executorService = Executors.newCachedThreadPool();
        //dashboard = HalDashboard.createInstance(this.telemetry);
        jewelServo = new FtcServo(this.hardwareMap, "jewelArm");
        jewelServo.setPosition(0.9);

        rightHinge = this.hardwareMap.get(Servo.class, "rightHinge");
        leftHinge = this.hardwareMap.get(Servo.class, "leftHinge");

        leftPickupServo = this.hardwareMap.get(CRServo.class, "leftPickup");
        rightPickupServo = this.hardwareMap.get(CRServo.class, "rightPickup");
        leftPickupServo2 = this.hardwareMap.get(CRServo.class, "leftPickup2");
        rightPickupServo2 = this.hardwareMap.get(CRServo.class, "rightPickup2");

        //change for new pickup mechanism
//        rightPickupServo.setDirection(Servo.Direction.REVERSE);

        wristServo1 = this.hardwareMap.get(CRServo.class, "wristServo1");
        wristServo1.setDirection(CRServo.Direction.REVERSE);
        wristServo2 = this.hardwareMap.get(CRServo.class, "wristServo2");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        relicServo = hardwareMap.get(Servo.class, "relicServo");
        //pickupHw.init(hardwareMap);

        //gyro = new SWIMUGyro(hardwareMap, "imu", null);
        //gyro.calibrate();

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
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
        driveBase.setBrakeMode(true);
        /*driveBase = new TrcDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMo tor, rightRearMotor);*/

        //gyro.setEnabled(true);

        gamepad = new SWGamePad("driver gamepad", gamepad1, 0.05F);
        gamepad.enableDebug(true);

        //driveBase.enableGyroAssist(gyroScale, gyroKp);


        /*
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
        */

        driveRunnable = new Runnable() {
            @Override
            public void run() {
                while (OP_MODE_IS_ACTIVE){
                    gamepad.setYInverted(setYInverted);
                    double rotation = -gamepad.getRightStickX();
                    magnitude = Range.clip(gamepad.getLeftStickMagnitude(), 0, 1);
                    if(turtleMode)
                        magnitude = magnitude/2;

                    double direction = gamepad.getLeftStickDirectionDegrees(true);

                    if(gamepad.getLeftStickX() == 0 && gamepad.getLeftStickY() == 0)
                        magnitude = 0;

                    //double addRotation = gyroScale*rotationRate;

                    //rotation += TrcUtil.clipRange(gyroKp*(rotation - addRotation));

                    //driveBase.mecanumDrive_XPolarFieldCentric(magnitude, direction, rotation);
                    driveBase.mecanumDrive_XPolar(magnitude, direction, rotation);
//                    jewelServo.setPosition(1.0);

                    if(gamepad1.a){
                        turtleMode = true;
                    } else if(gamepad1.b){
                        turtleMode = false;
                    }

                    if (gamepad1.x){
                        jewelServo.setPosition(1.0); // Jewel servo goes up
                    }

                    //telemetry.addData("Thread driveBase", "running");
                }
            }
        };

        armRunnable = new Runnable() {
            @Override
            public void run() {
                while (OP_MODE_IS_ACTIVE){
                    if(gamepad2.dpad_up){
                        armMotorSpeedLimiter = armMotorSpeedLimiter + 0.00001;
                    } else if(gamepad2.dpad_down){
                        armMotorSpeedLimiter = armMotorSpeedLimiter - 0.00001;
                    }

                    if (gamepad2.left_stick_y == 0) {
                        armMotor.setPower(gamepad2.right_stick_y*WristOrientation);
                    } else
                        armMotor.setPower(gamepad2.left_stick_y*armMotorSpeedLimiter*WristOrientation);

                    if (gamepad1.dpad_right){
                        //relicServo.setPosition(0.3);
                        relicServPos = 0.3;
                    } else if (gamepad1.dpad_left){
                        //relicServo.setPosition(0.7);
                        relicServPos = 0.8;
                    }

                    if (gamepad2.right_bumper){
                        WristOrientation = 1;
                    } else if (gamepad2.left_bumper) {
                        WristOrientation = -1;
                    }
                    relicServo.setPosition(relicServPos);
                }
            }
        };

        servoRunnable = new Runnable() {
            @Override
            public void run() {
                double servoPos = (float) 0.0;
                while (OP_MODE_IS_ACTIVE){
                    ///*
                    if (gamepad2.b) {
                        leftPickupServo.setPower(PickupHardware.MIN_FINGER_POS);
                        rightPickupServo.setPower(PickupHardware.MAX_FINGER_POS);
                        leftPickupServo2.setPower(PickupHardware.MIN_FINGER_POS);
                        rightPickupServo2.setPower(PickupHardware.MAX_FINGER_POS);
                    } //If touch sensor is pressed, stop wheels. If 'A' is pressed, run wheels. If neither is pressed, stop wheels
                    else if (!touchSensor.getState()) {
                        leftPickupServo.setPower(PickupHardware.LEFT_STOP_POS_1);
                        rightPickupServo.setPower(PickupHardware.RIGHT_STOP_POS_1);
                        leftPickupServo2.setPower(PickupHardware.LEFT_STOP_POS_2);
                        rightPickupServo2.setPower(PickupHardware.RIGHT_STOP_POS_2);
                    } //Turn inward
                    else if (gamepad2.a) {
                        leftPickupServo.setPower(PickupHardware.MAX_FINGER_POS);
                        rightPickupServo.setPower(PickupHardware.MIN_FINGER_POS);
                        leftPickupServo2.setPower(PickupHardware.MAX_FINGER_POS);
                        rightPickupServo2.setPower(PickupHardware.MIN_FINGER_POS);
                    } //Stop wheels
//                    else if (gamepad2.left_bumper) {
//                        leftPickupServo.setPosition(1.0);
//                        rightPickupServo.setPosition(0.5);
//                    }
//                    else if (gamepad2.right_bumper) {
//                        rightPickupServo.setPosition(-1.0);
//                        leftPickupServo.setPosition(0.53);
//                    }
                    else {
                        leftPickupServo.setPower(PickupHardware.LEFT_STOP_POS_1);
                        rightPickupServo.setPower(PickupHardware.RIGHT_STOP_POS_1);
                        leftPickupServo2.setPower(PickupHardware.LEFT_STOP_POS_2);
                        rightPickupServo2.setPower(PickupHardware.RIGHT_STOP_POS_2);
                    }
                    //*/

//                    //new pickup mechanism test
//                    if (gamepad2.b) {
//                        leftPickupServo.setPosition(0.1);
//                        rightPickupServo.setPosition(0.9);
//                    } //If touch sensor is pressed, stop wheels. If 'A' is pressed, run wheels. If neither is pressed, stop wheels
//                    else if (!touchSensor.getState()) {
//                        leftPickupServo.setPosition(0.5);
//                        rightPickupServo.setPosition(0.5);
//                    } //Turn inward
//                    else if (gamepad2.a) {
//                        leftPickupServo.setPosition(0.9);
//                        rightPickupServo.setPosition(0.1);
//                    } //Stop wheels
//                    else if (gamepad2.left_bumper) {
//                        leftPickupServo.setPosition(0.9);
//                        rightPickupServo.setPosition(0.5);
//                    }
//                    else if (gamepad2.right_bumper) {
//                        rightPickupServo.setPosition(0.1);
//                        leftPickupServo.setPosition(0.5);
//                    }
//                    else {
//                        leftPickupServo.setPosition(0.5);
//                        rightPickupServo.setPosition(0.5);
//                    }

                    //hinge servo
                    if(gamepad1.right_trigger > 0)
                        pressedTriggerR = true;
                    else pressedTriggerR = false;

                    if (pressedTriggerR) {
                        if(hingeUpR && prevPressedTriggerR != pressedTriggerR){
                            rightHingePos = PickupHardware.RIGHT_HINGE_DOWN;
                            hingeUpR = false;
                            //prevPressedTriggerR = pressedTriggerR;
                        } else if (prevPressedTriggerR != pressedTriggerR){
                            rightHingePos = PickupHardware.RIGHT_HINGE_UP;
                            hingeUpR = true;
                            //prevPressedTriggerR = pressedTriggerR;
                        }
                    }

                    rightHinge.setPosition(rightHingePos);

                    if(gamepad1.left_trigger > 0)
                        pressedTriggerL = true;
                    else pressedTriggerL = false;

                    if (pressedTriggerL) {
                        if(hingeUpL && prevPressedTriggerL != pressedTriggerL){
                            leftHingePos = PickupHardware.LEFT_HINGE_DOWN;
                            hingeUpL = false;
                            //prevPressedTriggerR = pressedTriggerR;
                        } else if (prevPressedTriggerL != pressedTriggerL){
                            leftHingePos = PickupHardware.LEFT_HINGE_UP;
                            hingeUpL = true;
                            //prevPressedTriggerR = pressedTriggerR;
                        }
                    }

                    leftHinge.setPosition(leftHingePos);

                    if (gamepad2.y) {
                        // Keep stepping up until we hit the max value.
                        position += INCREMENT ;
                        if (position >= MAX_POS ) {
                            position = MAX_POS;
                        }
                    }
                    else if(gamepad2.x){
                        // Keep stepping down until we hit the min value.
                        position -= INCREMENT ;
                        if (position <= MIN_POS ) {
                            position = MIN_POS;
                        }
                    } else {
                        position = 0;
                    }

                    setWristPosition(position*WristOrientation);

                }
            }
        };

    }

    private void initHardware(){

    }

    private void setWristPosition(double position) {
        wristServo1.setPower(position);
        wristServo2.setPower(position);
    }

    @Override
    public void loop() {
        if(!OP_MODE_IS_ACTIVE){
            OP_MODE_IS_ACTIVE = true;
            executorService.submit(driveRunnable);
            executorService.submit(armRunnable);
            executorService.submit(servoRunnable);
        }

        telemetry.addData("SLOW MODE", turtleMode);
        telemetry.addData("Servo Position", "%5.2f", position);
        telemetry.addData("rightHinge", rightHingePos);
        telemetry.addData("leftHinge", leftHingePos);
        telemetry.addData("relic servo", relicServPos);
        telemetry.addData("magnitude", magnitude);
        telemetry.addData("Arm Speed Limiter", armMotorSpeedLimiter);
        telemetry.addData("OP_MODE_IS_ACTIVE boolean", OP_MODE_IS_ACTIVE);
        telemetry.addData("ma z rotation rate", rotationRate);
        telemetry.addData("maxListSize", maxListSize);
        telemetry.update();

        prevPressedTriggerR = pressedTriggerR;
        prevPressedTriggerL = pressedTriggerL;
    }
}
