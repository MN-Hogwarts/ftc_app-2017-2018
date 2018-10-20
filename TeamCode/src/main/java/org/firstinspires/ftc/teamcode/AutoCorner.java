
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
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoOptions.AllianceColor;

import ftclib.FtcOpMode;

import static org.firstinspires.ftc.teamcode.AutoOptions.AngleMeasureHw.IMU;

            @Autonomous(name = "AutoCorner", group = "Main Auto")

//@Disabled

            public class AutoCorner extends FtcOpMode {

                // Define class members
                TankDriveMotors drivetrain;
                DcMotor eMotor = null;
                DcMotor rMotor = null;
                DcMotor lMotor = null;
                BNO055IMU imu;

                @Override
                public void initRobot() {
                    eMotor = hardwareMap.get(DcMotor.class, "eMotor");
                    rMotor = hardwareMap.get(DcMotor.class, "rMotor");
                    lMotor = hardwareMap.get(DcMotor.class, "lMotor");

//also the imu
//initialize the imu parameters too, it’s like 7 lines that look like BNO055IMU.
                    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
                    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                    parameters.loggingEnabled      = true;
                    parameters.loggingTag          = "IMU";
                    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                    imu = hardwareMap.get(BNO055IMU.class, "imu");
                    imu.initialize(parameters);

                    drivetrain.init(this);
                }

                @Override
                public void runOpMode() throws InterruptedException{

                    initRobot();
                    while(isStarted()) {
//telemetry
                    }
                    eMotor.setPower(1);
                    sleep(2000);
                    eMotor.setPower(0);
                    // de-elevate the robot
                    rMotor.setPower(1);
                    lMotor.setPower(1);
                    sleep(500);
                    // move forward an inch
                    drivetrain.turn(90, 0.4);
                    //imu
                    rMotor.setPower(1);
                    lMotor.setPower(1);
                    //drive the roomba forwards

                }
            }

