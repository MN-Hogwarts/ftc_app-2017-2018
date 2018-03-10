package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import ftclib.FtcDcMotor;
import swlib.SWGamePad;
import swlib.SwDriveBase;

@TeleOp(name = "Basic Mecanum 3", group = "teleop")
//@Disabled
public class BasicMecanum3 extends OpMode{

    private boolean OP_MODE_IS_ACTIVE = true;

    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;
    private SwDriveBase driveBase = null;
    private SWGamePad gamepad;
    private boolean setYInverted = true;
    private boolean turtleMode = false;
    private double magnitude = 0;

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

        driveBase = new SwDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
        driveBase.setBrakeMode(true);
        gamepad = new SWGamePad("driver gamepad", gamepad1);
        driveBase = new SwDriveBase(leftFrontMotor, leftRearMotor, rightRearMotor, rightRearMotor);
    }
    @Override
    public void loop() {
        gamepad.setYInverted(setYInverted);
        double rotation = -gamepad.getRightStickX();
        magnitude = Range.clip(gamepad.getLeftStickMagnitude(), 0, 1);
        if(turtleMode)
            magnitude = magnitude/2;

        double direction = gamepad.getLeftStickDirectionDegrees(true);

        if(gamepad.getLeftStickX() == 0 && gamepad.getLeftStickY() == 0)
            magnitude = 0;

        driveBase.mecanumDrive_XPolar(magnitude, direction, rotation);

        if(gamepad1.a){
            turtleMode = true;
        } else if(gamepad1.b){
            turtleMode = false;
        }
    }

}
