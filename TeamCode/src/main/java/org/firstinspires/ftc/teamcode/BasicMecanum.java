package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.LinkedList;
import java.util.List;

import ftclib.FtcDcMotor;
import ftclib.FtcServo;
import swlib.SWGamePad;
import swlib.SWIMUGyro;
import swlib.SwDriveBase;
import trclib.TrcSensor;
import trclib.TrcServo;
import trclib.TrcUtil;

@TeleOp(name = "Basic Mecanum", group = "teleop")
public class BasicMecanum extends OpMode{

    private boolean OP_MODE_IS_ACTIVE = true;

    private FtcDcMotor armMotor;
    private double armMotorSpeedLimiter = 0.3;
    private SwDriveBase driveBase = null;
    private SWIMUGyro gyro = null;
    private SWGamePad gamepad;
    //private HalDashboard dashboard = null;
    private boolean setYInverted = true;
    private TrcServo jewelServo = null;
    private Servo wristServo, hingeServo;
    private Servo leftPickupServo, rightPickupServo;
    private Servo relicServo;
    private DigitalChannel touchSensor ;
    //private boolean adjustGyroScale = true;
    private double relicServPos = 0.7;

    private static double gyroKp = 0;
    private static double gyroScale = 1.;
    private static List<TrcSensor.SensorData<Double>> maRotationRate = new LinkedList<>();
    private static int maxListSize = 10;
    private static double rotationRate = 0;

    static final double INCREMENT   = 0.3;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.9;     // Maximum rotational position
    static final double MIN_POS     =  0.2;     // Minimum rotational position

    private boolean hingeUp = true;
    private boolean pressedTrigger = false;
    private boolean prevPressedTrigger = false;

    double  position = 0.5;
    double  hingePosition = 0.5;


    private boolean turtleMode = false;
    private double magnitude = 0;
    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;

    @Override
    public void init() {

        leftFrontMotor = new FtcDcMotor(this.hardwareMap, "leftFront", null, null);
        leftRearMotor = new FtcDcMotor(this.hardwareMap, "leftRear", null, null);
        rightFrontMotor = new FtcDcMotor(this.hardwareMap, "rightFront", null, null);
        rightRearMotor = new FtcDcMotor(this.hardwareMap, "rightRear", null, null);

        leftFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    @Override
    public void loop() {
        double r = Math.hypot(gamepad.getLeftStickY(), gamepad.left_stick_x);
        double robotAngle = Math.atan2(gamepad.left_stick_x, gamepad.left_stick_y) - Math.PI / 4;
        double rightX = -gamepad.right_stick_y;

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
        // jewelServo.setPosition(0.9);

        if(gamepad.a){
            turtleMode = true;
        } else if(gamepad.b){
            turtleMode = false;
        }
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFrontMotor.setPower(v1*0.5);
        rightFrontMotor.setPower(v2*0.5);
        leftRearMotor.setPower(v3*0.5);
        rightRearMotor.setPower(v4*0.5);

    }

}

