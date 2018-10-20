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

package teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.VisitorPattern.EmptyVisitable;
import org.firstinspires.ftc.teamcode.VisitorPattern.VisitableActions;
import org.firstinspires.ftc.teamcode.VisitorPattern.Visitor;

import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
import ftclib.FtcBNO055Imu;
import trclib.TrcDriveBase;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 */
public class MecanumDriveBase implements Visitor {
    /* Public OpMode members. */
    FtcDcMotor leftFrontMotor   = null;
    FtcDcMotor rightFrontMotor  = null;
    FtcDcMotor leftBackMotor    = null;
    FtcDcMotor rightBackMotor   = null;

    FtcBNO055Imu imu = null;

    TrcDriveBase mecanumDrive   = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    FtcOpMode linearFtcOpMode = null;

    double startangle   = 0;

    double maxTurnPower = 0.8;
    double turnPower2   = 0.7;
    double turnPower3   = 0.65;
    double minTurnPower = 0.55;

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = new FtcBNO055Imu(hwMap, "imu");
        imu.imu.initialize(parameters);

        mecanumDrive = new TrcDriveBase(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor,
                imu.gyro);
        // Define and initialize ALL installed servos.
    }

    void setAllMotorsRunMode(DcMotor.RunMode runMode) {
        leftFrontMotor.motor.setMode(runMode);
        leftBackMotor.motor.setMode(runMode);
        rightFrontMotor.motor.setMode(runMode);
        rightBackMotor.motor.setMode(runMode);
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

        double startAngle = getAngleX();
        double angleZ = getAngleX();

        double angDiff = (turnAngle - angleZ) % 360;
        if (360 - Math.abs(angDiff) < Math.abs(angDiff))
            angDiff = -(360 * Math.signum(angDiff) - angDiff);

        linearFtcOpMode.telemetry.log().add("Angle Difference: " + angDiff);
        linearFtcOpMode.telemetry.update();

        if (angDiff < 0) { //turns right

            while (linearFtcOpMode.opModeIsActive() && angDiff < 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                linearFtcOpMode.telemetry.update();

                turnRightWithoutAngle(power);
                linearFtcOpMode.telemetry.addData("Power", leftFrontMotor.getPower());

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        } else if (angDiff > 0) { //turns left

            while (linearFtcOpMode.opModeIsActive() && angDiff > 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                linearFtcOpMode.telemetry.update();

                turnLeftWithoutAngle(power);

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        }

        mecanumDrive.stop();
    }

    public void turn(int turnAngle) throws InterruptedException {
        turn(turnAngle, new EmptyVisitable(), "");
    }

    public void turn(int turnAngle, VisitableActions visAct, String methodName) throws InterruptedException {
        if (linearFtcOpMode == null) {
            return;
        }

        double startAngle = getAngleX();
        double angleZ = getAngleX();

        double angDiff = (turnAngle - angleZ) % 360;
        if (360 - Math.abs(angDiff) < Math.abs(angDiff))
            angDiff = -(360 * Math.signum(angDiff) - angDiff);

        linearFtcOpMode.telemetry.log().add("Angle Difference: " + angDiff);
        linearFtcOpMode.telemetry.update();

        if (angDiff < 0) { //turns right

            while (linearFtcOpMode.opModeIsActive() && angDiff < 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                visit(visAct, methodName);
                linearFtcOpMode.telemetry.update();

                turnRightWithoutAngle(turnPower(angDiff));
                linearFtcOpMode.telemetry.addData("Power", leftFrontMotor.getPower());

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        } else if (angDiff > 0) { //turns left

            while (linearFtcOpMode.opModeIsActive() && angDiff > 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                visit(visAct, methodName);
                linearFtcOpMode.telemetry.update();

                turnLeftWithoutAngle(turnPower(angDiff));

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        }

        mecanumDrive.stop();
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

    public void setTurnPowers(double maxPower, double power2, double power3, double minPower) {
        maxTurnPower    = maxPower;
        turnPower2      = power2;
        turnPower3      = power3;
        minTurnPower    = minPower;
    }

    private double turnPower(double difference) {
        if (Math.abs(difference) < 20) {
            return minTurnPower;
        } else if (Math.abs(difference) < 45) {
            return turnPower3;
        } else if (Math.abs(difference) < 90) {
            return turnPower2;
        } else return maxTurnPower;
    } // Adjust powers depending on robot

    public void leftSideSwingTurn(int turnAngle, VisitableActions visAct, String methodName) throws InterruptedException {
        if (linearFtcOpMode == null) {
            return;
        }

        double startAngle = getAngleX();
        double angleZ = getAngleX();

        double angDiff = (turnAngle - angleZ) % 360;
        if (360 - Math.abs(angDiff) < Math.abs(angDiff))
            angDiff = -(360 * Math.signum(angDiff) - angDiff);

        linearFtcOpMode.telemetry.log().add("Angle Difference: " + angDiff);
        linearFtcOpMode.telemetry.update();

        if (angDiff < 0) { //turns right

            while (linearFtcOpMode.opModeIsActive() && angDiff < 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                visit(visAct, methodName);
                linearFtcOpMode.telemetry.update();

                leftFrontMotor.setPower(turnPower(angDiff));
//                rightFrontMotor.setPower(-turnPower(angDiff));
                leftBackMotor.setPower(turnPower(angDiff));
//                rightBackMotor.setPower(-turnPower(angDiff));

                linearFtcOpMode.telemetry.addData("Power", leftFrontMotor.getPower());

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        } else if (angDiff > 0) { //turns left

            while (linearFtcOpMode.opModeIsActive() && angDiff > 0) {

                angleZ = getAngleX();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                linearFtcOpMode.telemetry.addData("Angle", angleZ);
                linearFtcOpMode.telemetry.addData("Difference", angDiff);
                visit(visAct, methodName);
                linearFtcOpMode.telemetry.update();

                leftFrontMotor.setPower(-turnPower(angDiff));
//                rightFrontMotor.setPower(turnPower(angDiff));
                leftBackMotor.setPower(-turnPower(angDiff));
//                rightBackMotor.setPower(turnPower(angDiff));

                linearFtcOpMode.telemetry.addData("Power", leftFrontMotor.getPower());

                linearFtcOpMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        }

        mecanumDrive.stop();
    }

    public void rightSideSwingTurn(int turnAngle, VisitableActions visAct, String methodName) throws InterruptedException {
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
                visit(visAct, methodName);
                linearFtcOpMode.telemetry.update();

//                leftFrontMotor.setPower(turnPower(angDiff));
                rightFrontMotor.setPower(-turnPower(angDiff));
//                leftBackMotor.setPower(turnPower(angDiff));
                rightBackMotor.setPower(-turnPower(angDiff));

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
                visit(visAct, methodName);
                linearFtcOpMode.telemetry.update();

//                leftFrontMotor.setPower(-turnPower(angDiff));
                rightFrontMotor.setPower(turnPower(angDiff));
//                leftBackMotor.setPower(-turnPower(angDiff));
                rightBackMotor.setPower(turnPower(angDiff));

                linearFtcOpMode.telemetry.addData("Power", leftFrontMotor.getPower());

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

    void setStartangle(double angle) {
        startangle = angle;
    }

    double getAngleX() {
        return imu.imu.getAngularOrientation().firstAngle + startangle;
    }

    @Override
    public void visit(VisitableActions visAct, String method) {
        visAct.accept(method);
    }
}

