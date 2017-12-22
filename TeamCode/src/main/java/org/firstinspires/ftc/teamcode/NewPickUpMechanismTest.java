package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by spmega4567 on 12/21/17.
 */
@TeleOp(name = "New Pickup Mechanism Test", group = "test")
public class NewPickUpMechanismTest extends OpMode {
    private Servo leftPickupServo, rightPickupServo;

    static final double INCREMENT   = 0.0005;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    @Override
    public void init() {
        leftPickupServo = this.hardwareMap.get(Servo.class, "leftPickup");
        rightPickupServo = this.hardwareMap.get(Servo.class, "rightPickup");
        rightPickupServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(getRuntime() < 8){
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );

            // Set the servo to the new position and pause;
            leftPickupServo.setPosition(position);
            rightPickupServo.setPosition(position);
        } else {
            leftPickupServo.setPosition(0.5);
            rightPickupServo.setPosition(0.5);

            telemetry.addData("Stopping the servos:", "Stopped");
        }

        telemetry.addData("runtime", getRuntime());
        telemetry.update();

    }

    @Override
    public void stop() {
        //note: for some reason, this method IS NOT CALLED
    }
}
