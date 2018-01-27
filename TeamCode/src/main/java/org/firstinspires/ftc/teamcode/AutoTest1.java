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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoOptions.AllianceColor;

import ftclib.FtcOpMode;

import static org.firstinspires.ftc.teamcode.AutoOptions.AngleMeasureHw.IMU;

/**
 * Rotates Servo between min and max position or rotatescontinuaou
 */
@Autonomous(name = "Auto 1", group = "Concept")
//@Disabled
public class AutoTest1 extends FtcOpMode {

    AutonomousActions auto = new AutonomousActions();

    // Define class members

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
            telemetry.addData("Color Sensor blue", auto.jewelColorL.blue());
            telemetry.addData("Color Sensor red", auto.jewelColorL.red());
            telemetry.addData("Right Bottom Blue", auto.tapeSensorR.blue());
            telemetry.addData("Right Bottom Red", auto.tapeSensorR.red());
            telemetry.addData("Left Bottom Blue", auto.tapeSensorL.blue());
            telemetry.addData("Left Bottom Red", auto.tapeSensorL.red());
            telemetry.addData("Angle X", auto.getAngleX());
            telemetry.addData("Angle Y", auto.getAngleY());
            telemetry.addData("Angle Z", auto.getAngleZ());
            telemetry.addData("Left distance", auto.leftRange.getDistance(DistanceUnit.CM));
            telemetry.addData("Right distance", auto.rightRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        //auto.pictographID();
        //auto.jewelColor();
        //auto.distanceToWall1();
        //sleep(5000);
        //auto.driveToCryptobox();
//        auto.firstTapeFound = true;
//        auto.positionUsingTape();
//        auto.cryptoboxAngleCorrection();
//        auto.mecanumDriveBase.mecanumDrive.mecanumDrive_BoxPolar(0.5, 0, -1); // -1 is left
        auto.tapeMap.put(auto.inSensInTape, true);
        auto.diagonalAlignment();
//        auto.moveFWBW();
        while (opModeIsActive()) {
            telemetry.addData("Left encoder position:", auto.mecanumDriveBase.leftBackMotor.motor.getCurrentPosition());
            telemetry.addData("Right encoder position:", auto.mecanumDriveBase.rightBackMotor.motor.getCurrentPosition());
//            telemetry.update();
        }

        //auto.initVuforia();
        //auto.pictographID(); //run Vuforia method, includes initVuforia()

    }
}
