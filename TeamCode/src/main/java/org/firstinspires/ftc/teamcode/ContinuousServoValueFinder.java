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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo power each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo power is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "ContinuousServoValueFinder", group = "Concept")
//@Disabled
public class ContinuousServoValueFinder extends OpMode {

    double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational power
    static final double MIN_POS     = -1.0;     // Minimum rotational power

    // Define class members
    CRServo servo;
    double power = (MAX_POS - MIN_POS) / 2; // Start at halfway power
    boolean rampUp = true;

    boolean prevA   = false;
    boolean prevX   = false;
    boolean prevB   = false;
    boolean prevY   = false;

    @Override
    public void init() {

        servo = hardwareMap.get(CRServo.class, "leftPickup");

        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();

    }

    @Override
    public void loop() {

        if (gamepad1.a && !prevA) {
            // Keep stepping up until we hit the max value.
            power += INCREMENT ;
            if (power >= MAX_POS ) {
                power = MAX_POS;
            }
            prevA = true;
            prevX = false;
        }
        else if (gamepad1.x && !prevX){
            // Keep stepping down until we hit the min value.
            power -= INCREMENT ;
            if (power <= MIN_POS ) {
                power = MIN_POS;
            }
            prevA = false;
            prevX = true;
        }
        else {
            prevA = false;
            prevX = false;
        }

        if (gamepad1.b && !prevB) {
            // Keep stepping up until we hit the max value.
            INCREMENT += 0.005;

            prevB = true;
            prevY = false;
        }
        else if (gamepad1.y && !prevY){
            // Keep stepping down until we hit the min value.
            INCREMENT -= 0.005;

            prevB = false;
            prevY = true;
        }
        else {
            prevB = false;
            prevY = false;
        }

        servo.setPower(power);

        // Display the current value
        telemetry.addData("Servo Speed", "%5.2f", power);
        telemetry.addData("Increment", INCREMENT);

        telemetry.addLine("Gamepad 1");
        telemetry.addLine("A: increase servo power");
        telemetry.addLine("X: decrease servo power");
        telemetry.addLine("B: increase increment");
        telemetry.addLine("Y: decrease increment");
        telemetry.update();

    }
}
