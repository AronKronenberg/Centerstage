package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Intake Tester")
public class IntakeTester extends LinearOpMode {

    // For right intake: 0.4 = open, 0.659 = closed
    // For left intake 0.6 = open, 0.415 = closed

    public static double rightPos = 0;
    public static double leftPos = 0;

    Servo rightIntake;
    Servo leftIntake;

    @Override
    public void runOpMode() throws InterruptedException {
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");

        waitForStart();

        while (opModeIsActive()) {
            rightIntake.setPosition(rightPos);
            leftIntake.setPosition(leftPos);
        }

    }
}
