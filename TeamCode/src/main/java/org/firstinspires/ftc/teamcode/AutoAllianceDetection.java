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
import com.qualcomm.robotcore.hardware.Servo;

import ftclib.FtcOpMode;

import static org.firstinspires.ftc.teamcode.AngleMeasureHw.IMU;

/**
 * Rotates Servo between min and max position or rotatescontinuaou
 */
@Autonomous(name = "Auto Alliance Detection", group = "Concept")
//@Disabled
public class AutoAllianceDetection extends FtcOpMode {

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
        Servo relicServo = hardwareMap.get(Servo.class, "relicServo");
        relicServo.setPosition(0.7);

        auto.initOpmode(this);
        auto.initMecanum();

        auto.initJewelHardware(IMU);
        auto.initGlyphHardware();
        auto.initAlliance();

        while (!isStarted()) {
            telemetry.addLine("Hi");
            telemetry.addData("Color Sensor blue", auto.colorSensor.blue());
            telemetry.addData("Color Sensor red", auto.colorSensor.red());
            telemetry.addData("Angle X", auto.getAngleX());
            telemetry.addData("Angle Y", auto.getAngleY());
            telemetry.addData("Angle Z", auto.getAngleZ());

            telemetry.addData("Right Bottom BLUE", auto.tapeSensorR.blue());
            telemetry.addData("Left Bottom BLUE", auto.tapeSensorL.blue());
            telemetry.addData("Right Bottom RED", auto.tapeSensorR.red());
            telemetry.addData("Left Bottom RED", auto.tapeSensorL.red());

            if(auto.allianceColor == auto.allianceColor.RED){
                telemetry.addLine("Alliance: RED");
            }
            else if(auto.allianceColor == auto.allianceColor.BLUE){
                telemetry.addLine("Alliance: BLUE");
            }
            else{
                telemetry.addLine("ERROR");
            }

            telemetry.update();
    }

        auto.jewelColor();
        auto.mecanumDriveBase.mecanumDrive.mecanumDrive_XPolar(1.0, -90, 0);
        sleep(1500);
        auto.mecanumDriveBase.mecanumDrive.stop();
    }
}