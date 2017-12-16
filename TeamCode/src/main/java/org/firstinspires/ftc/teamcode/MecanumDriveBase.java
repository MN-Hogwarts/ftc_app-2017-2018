/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
import swlib.SWIMUGyro;
import swlib.SwDriveBase;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 */
public class MecanumDriveBase
{
    /* Public OpMode members. */
    FtcDcMotor leftFrontMotor   = null;
    FtcDcMotor rightFrontMotor  = null;
    FtcDcMotor leftBackMotor    = null;
    FtcDcMotor rightBackMotor   = null;

    SWIMUGyro gyro              = null;
    BNO055IMU imu               = null;

    SwDriveBase mecanumDrive   = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    FtcOpMode linearFtcOpMode = null;

    /* Constructor */
    public MecanumDriveBase(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        init(ahwMap, null);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, FtcOpMode opMode) {
    // Save reference to Hardware map
        linearFtcOpMode = opMode;

        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = new FtcDcMotor("leftFront");
        rightFrontMotor = new FtcDcMotor("rightFront");
        leftBackMotor = new FtcDcMotor("leftRear");
        rightBackMotor = new FtcDcMotor("rightRear");

        gyro = new SWIMUGyro(hwMap, "imu", null);
        imu = hwMap.get(BNO055IMU.class, "imu");
        gyro.calibrate();

        /*
        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

        mecanumDrive = new SwDriveBase(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor,
                gyro);
        // Define and initialize ALL installed servos.
    }

    void setBrakeModeEnabled(boolean brake) {
        leftFrontMotor.setBrakeModeEnabled(brake);
        leftBackMotor.setBrakeModeEnabled(brake);
        rightFrontMotor.setBrakeModeEnabled(brake);
        rightBackMotor.setBrakeModeEnabled(brake);
    }

    public void turn(int turnAngle, double power) throws InterruptedException {
        if (linearFtcOpMode == null) {
            return;
        }

        // leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // int leftPos = leftMotor.getCurrentPosition();
        // int rightPos = rightMotor.getCurrentPosition();

        double startAngle = getAngleX();
        double angleZ = getAngleX();

        double angDiff = (turnAngle - angleZ) % 360;
        if (360 - Math.abs(angDiff) < Math.abs(angDiff))
            angDiff = -(360 * Math.signum(angDiff) - angDiff);

        linearFtcOpMode.telemetry.log().add("Angle Difference: " + angDiff);
        linearFtcOpMode.telemetry.update();

        if (angDiff < 0) { //turns right
            //leftMotor.setPower(APPROACH_SPEED * .6 );
            //rightMotor.setPower(-APPROACH_SPEED * .6);

            while (linearFtcOpMode.opModeIsActive() && angDiff < 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                linearFtcOpMode.telemetry.update();

                turnRightWithoutAngle(power);
                /*
                leftFrontMotor.setPower(turnPower(angDiff));
                rightFrontMotor.setPower(-turnPower(angDiff));
                leftBackMotor.setPower(turnPower(angDiff));
                rightBackMotor.setPower(-turnPower(angDiff));
                */
                linearFtcOpMode.telemetry.addData("Power", leftFrontMotor.getPower());

                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, -90, false);
                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, angDiff);

                /* if (leftMotor.getCurrentPosition() - 100 > leftPos
                        && rightMotor.getCurrentPosition() + 100 < rightPos
                        && IMUheading() == startAngle) {
                    resetIMuandPos(leftPos, rightPos);
                } */

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        } else if (angDiff > 0) { //turns left
            //leftMotor.setPower(-APPROACH_SPEED);
            //rightMotor.setPower(APPROACH_SPEED);

            while (linearFtcOpMode.opModeIsActive() && angDiff > 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                linearFtcOpMode.telemetry.update();

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

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        }

        mecanumDrive.stop();
        /*
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        */
    }

    public void turn(int turnAngle) throws InterruptedException {
        if (linearFtcOpMode == null) {
            return;
        }

        // leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // int leftPos = leftMotor.getCurrentPosition();
        // int rightPos = rightMotor.getCurrentPosition();

        double startAngle = getAngleX();
        double angleZ = getAngleX();

        double angDiff = (turnAngle - angleZ) % 360;
        if (360 - Math.abs(angDiff) < Math.abs(angDiff))
            angDiff = -(360 * Math.signum(angDiff) - angDiff);

        linearFtcOpMode.telemetry.log().add("Angle Difference: " + angDiff);
        linearFtcOpMode.telemetry.update();

        if (angDiff < 0) { //turns right
            //leftMotor.setPower(APPROACH_SPEED * .6 );
            //rightMotor.setPower(-APPROACH_SPEED * .6);

            while (linearFtcOpMode.opModeIsActive() && angDiff < 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                linearFtcOpMode.telemetry.update();

                turnRightWithoutAngle(turnPower(angDiff));
                /*
                leftFrontMotor.setPower(turnPower(angDiff));
                rightFrontMotor.setPower(-turnPower(angDiff));
                leftBackMotor.setPower(turnPower(angDiff));
                rightBackMotor.setPower(-turnPower(angDiff));
                */
                linearFtcOpMode.telemetry.addData("Power", leftFrontMotor.getPower());

                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, -90, false);
                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, angDiff);

                /* if (leftMotor.getCurrentPosition() - 100 > leftPos
                        && rightMotor.getCurrentPosition() + 100 < rightPos
                        && IMUheading() == startAngle) {
                    resetIMuandPos(leftPos, rightPos);
                } */

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        } else if (angDiff > 0) { //turns left
            //leftMotor.setPower(-APPROACH_SPEED);
            //rightMotor.setPower(APPROACH_SPEED);

            while (linearFtcOpMode.opModeIsActive() && angDiff > 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                linearFtcOpMode.telemetry.update();

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

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        }

        mecanumDrive.stop();
        /*
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        */
    }

    public void turnLeftWithoutAngle(double power) {
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(power);
    }

    public void turnRightWithoutAngle(double power) {
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(-power);
    }

    private double turnPower(double difference) {
        if (Math.abs(difference) < 20) {
            //return 0.15;
            return 0.55;
        } else if (Math.abs(difference) < 45) {
            //return 0.3;
            return 0.65;
        } else if (Math.abs(difference) < 90) {
            return 0.7;
        } else return 0.8;
    }

    double getAngleX() {
        return imu.getAngularOrientation().firstAngle;
    }
}

