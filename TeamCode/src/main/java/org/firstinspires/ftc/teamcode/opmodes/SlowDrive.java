package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="Slow Drive")
public class SlowDrive extends LinearOpMode {
    Hardware robot = new Hardware();
    SampleMecanumDrive drive;

    public static final double driveSpeed = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            double y = -gamepad1.left_stick_y; // forward/backward
            double x = gamepad1.left_stick_x * 1.1; // right/left
            double rx = gamepad1.right_stick_x; // clockwise/counterclockwise

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.frontLeft.setPower(frontLeftPower * driveSpeed);
            robot.frontRight.setPower(frontRightPower * driveSpeed);
            robot.backLeft.setPower(backLeftPower * driveSpeed);
            robot.backRight.setPower(backRightPower * driveSpeed);
        }
    }
}
