package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by spmega4567 on 10/13/17.
 */
@Autonomous(name = "MecMotorDiagnosticsLinear", group = "Test")
public class MecMotorDiagnosticsLinear extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftRearMotor = this.hardwareMap.dcMotor.get("leftRear");
        DcMotor leftFrontMotor = this.hardwareMap.dcMotor.get("leftFront");
        DcMotor rightRearMotor = this.hardwareMap.dcMotor.get("rightRear");
        DcMotor rightFrontMotor = this.hardwareMap.dcMotor.get("rightFront");

        leftRearMotor.setPower(0.5);
        sleep(1500);
        leftRearMotor.setPower(0);

        leftFrontMotor.setPower(0.5);
        sleep(1500);
        leftFrontMotor.setPower(0);

        rightRearMotor.setPower(0.5);
        sleep(1500);
        rightRearMotor.setPower(0);

        rightFrontMotor.setPower(0.5);
        sleep(1500);
        rightFrontMotor.setPower(0);

    }

    private void sleep(int millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
