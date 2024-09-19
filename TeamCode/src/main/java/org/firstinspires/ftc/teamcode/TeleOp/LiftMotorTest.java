package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="Lift test")
public class LiftMotorTest extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "lift motor");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(0.5);
            sleep(10000);
            motor.setPower(0);
        }
    }
}
