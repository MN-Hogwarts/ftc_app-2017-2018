package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import ftclib.FtcDcMotor;

@TeleOp(name = "Basic Mecanum 2", group = "teleop")
//@Disabled
public class BasicMecanum2 extends OpMode{

    private boolean OP_MODE_IS_ACTIVE = true;

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
        double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double robotAngle = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;

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
