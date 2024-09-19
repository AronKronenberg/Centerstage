package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Reset")
public class ResetServo extends LinearOpMode {
    Servo leftServo;
    Servo rightServo;

    @Override
    public void runOpMode() throws InterruptedException {
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Servo position", leftServo.getPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                leftServo.setPosition(0);
                rightServo.setPosition(0);
            }
            if (gamepad1.a) {
                leftServo.setPosition(0.25);
                rightServo.setPosition(-0.25);
            }

            telemetry.addData("Reseted servo position", leftServo.getPosition());
            telemetry.update();
        }

        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }
}
