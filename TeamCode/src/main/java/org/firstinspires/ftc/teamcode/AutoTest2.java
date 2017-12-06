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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import ftclib.FtcOpMode;

import static org.firstinspires.ftc.teamcode.AngleMeasureHw.IMU;

/**
 * Rotates Servo between min and max position or rotatescontinuaou
 */
@Autonomous(name = "Auto 2", group = "Concept")
//@Disabled
public class AutoTest2 extends FtcOpMode {

    AutonomousActions auto = new AutonomousActions();

    // Define class members
    Servo   servo;
    double  position = .55;//(MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;


    @Override
    public void initRobot() {

    }

    @Override
    public void runOpMode() throws InterruptedException{

        auto.initOpmode(this);
        auto.initMecanum();
        //auto.initVuforia();
        auto.initAlliance(AllianceColor.RED);
        auto.initJewelHardware(IMU);
        auto.initGlyphHardware();
        while (!isStarted()) {
            telemetry.addLine("Hi");
            telemetry.addData("Color Sensor blue", auto.colorSensor.blue());
            telemetry.addData("Color Sensor red", auto.colorSensor.red());
            telemetry.addData("Bottom Sensor blue", auto.tapeSensorL.blue());
            telemetry.addData("Bottom Sensor red", auto.tapeSensorL.red());
            telemetry.addData("Angle X", auto.getAngleX());
            telemetry.addData("Angle Y", auto.getAngleY());
            telemetry.addData("Angle Z", auto.getAngleZ());
            //telemetry.addData("Left distance", auto.leftRange.getDistance(DistanceUnit.CM));
            //telemetry.addData("Right distance", auto.rightRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        //auto.jewelColor();
        //auto.glyphPickup();
//        auto.encoderDrive(0.6, 700, 2);
//        auto.mecanumDriveBase.leftBackMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        auto.mecanumDriveBase.rightBackMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        auto.positionUsingTape();

        //sleep(5000);
        while (opModeIsActive()) {
            telemetry.addData("Left encoder position:", auto.mecanumDriveBase.leftBackMotor.motor.getCurrentPosition());
            telemetry.addData("Right encoder position:", auto.mecanumDriveBase.rightBackMotor.motor.getCurrentPosition());
            telemetry.update();
        }

        //auto.initVuforia();
        //auto.pictographID(); //run Vuforia method, includes initVuforia()

    }
}
