/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="FourServo", group="Pushbot")
//@Disabled
public class FourServo extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    Servo leftServo ;
    Servo rightServo ;
    Servo left_Servo ;
    Servo right_Servo ;
    DigitalChannel touchSensor ;
    double  position = .55;//(MAX_POS - MIN_POS) / 2; // Start at halfway position
    static final int    CYCLE_MS    =   2000;     // period of each cycle
    // could also use HardwarePushbotMatrix class.
   /* double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo */


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        leftServo = hardwareMap.get(Servo.class, "leftWheel") ;
        rightServo = hardwareMap.get(Servo.class, "rightWheel") ;
        left_Servo = hardwareMap.get (Servo.class, "left_Wheel") ;
        right_Servo = hardwareMap.get (Servo.class, "right_Wheel") ;
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor") ;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);

        // Run wheels backwards if 'B' is pressed. If not, stop servos
    if (gamepad1.b) {
        leftServo.setPosition(-1.0);
        rightServo.setPosition(1.0);
        left_Servo.setPosition(-1.0);
        right_Servo.setPosition(1.0);
        telemetry.addData("Right Servo Value", right_Servo.getPosition());
        telemetry.update();
    } //If touch sensor is pressed, stop wheels. If 'A' is pressed, run wheels. If neither is pressed, stop wheels
    else if (!touchSensor.getState()) {
        leftServo.setPosition(0.53);
        rightServo.setPosition(0.5);
        left_Servo.setPosition(0.52);
        right_Servo.setPosition(0.5);
    telemetry.addData("Left Servo Value", left_Servo.getPosition());
    telemetry.update() ;
    }
    else if (gamepad1.a) {
        leftServo.setPosition(1.0);
        rightServo.setPosition(-1.0);
        left_Servo.setPosition(1.0);
        right_Servo.setPosition(-1.0);
    telemetry.addData("Left Servo Value", left_Servo.getPosition()) ;
    telemetry.update() ;
    }
     else {
        leftServo.setPosition(0.53);
        rightServo.setPosition(0.5);
        left_Servo.setPosition(0.52);
        right_Servo.setPosition(0.5);
     telemetry.addData("Left Servo Value", left_Servo.getPosition()) ;
     telemetry.update() ;
     }

        telemetry.addData("Touch sensor value", touchSensor.getState());
        telemetry.update();


        }

    @Override
    public void stop() {
    }

}