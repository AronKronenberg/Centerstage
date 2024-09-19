package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="Elbow Tester")
public class ElbowTesting extends LinearOpMode {
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.elbow.setPosition(1/180.0);
            }
            if (gamepad1.b) {
                robot.elbow.setPosition(30/180.0);
            }
            if (gamepad1.y) {
                robot.elbow.setPosition(150/180.0);
            }
            if (gamepad1.x) {
                robot.elbow.setPosition(170/180.0);
            }
        }
    }
}
