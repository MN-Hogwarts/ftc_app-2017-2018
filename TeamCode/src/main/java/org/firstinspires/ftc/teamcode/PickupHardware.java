package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftclib.FtcDcMotor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class PickupHardware
{
    /* Public OpMode members. */
    CRServo leftServo ;
    CRServo rightServo ;
    CRServo wristServo1;
    CRServo wristServo2;
    CRServo left_Servo ;
    CRServo right_Servo ;
    Servo rightHinge;
    Servo leftHinge;
    DigitalChannel touchSensor ;
    FtcDcMotor armMotor;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    static final double MAX_FINGER_POS = 0.8;
    static final double MIN_FINGER_POS = -0.8;
    static final double LEFT_STOP_POS   = 0.025;
    static final double RIGHT_STOP_POS   = 0;

    public static final double RIGHT_HINGE_UP   = 0.25;
    public static final double RIGHT_HINGE_DOWN = 0.95;
    public static final double LEFT_HINGE_UP    = 1.0;
    public static final double LEFT_HINGE_DOWN  = 0.2;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public PickupHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        Log.d("Hardware Map", "Initialized");
        // Define and Initialize Motors
        leftServo = hwMap.get(CRServo.class, "leftPickup") ;
        Log.d("Left Pickup", "Initialized");
        rightServo = hwMap.get(CRServo.class, "rightPickup") ;
        Log.d("Right Pickup", "Initialized");
        wristServo1 = hwMap.get(CRServo.class, "wristServo1") ;
        wristServo1.setDirection(CRServo.Direction.REVERSE);
        wristServo2 = hwMap.get(CRServo.class, "wristServo2") ;
        Log.d("Wrist Servo", "Initialized");
        armMotor = new FtcDcMotor("armMotor");
        Log.d("Arm Motor", "Initialized");
        touchSensor = hwMap.get(DigitalChannel.class, "touchSensor") ;
        Log.d("Touch Sensor", "Initialized");
        rightHinge = hwMap.get(Servo.class, "rightHinge") ;
        leftHinge = hwMap.get(Servo.class, "leftHinge") ;
    }

    public void setWristPosition(double position) {
        wristServo1.setPower(position);
        wristServo2.setPower(position);
    }
}

