package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.DriveConstants.LOGO_FACING_DIR;
import static org.firstinspires.ftc.teamcode.DriveConstants.USB_FACING_DIR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="TeleOp")
public class SAR2D2TeleOp extends LinearOpMode {
    Hardware robot = new Hardware();
    Controller mainPad;
    Controller secondaryPad;

    private PIDController controller;

    public static double p = 0.065, i = 0, d = 0;
    public static double f = -0.1;
    public static int target = 0;

    SampleMecanumDrive drive;

    boolean robotCentric = false;

    YawPitchRollAngles angles;

    Hardware.DriveSpeed driveSpeedMode = Hardware.DriveSpeed.NORMAL;

    double liftSpeed = Hardware.LIFT_SPEED; // current lift speed of the robot

    boolean leftIntakeOpen = false;
    boolean rightIntakeOpen = false;

    boolean doTipCorrection = true;

    double previousTime = 0;

    boolean reverseDrive = false;

    boolean alignmentControl = false;

    Servo rightIntake;
    Servo leftIntake;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        drive = new SampleMecanumDrive(hardwareMap);
        mainPad = new Controller(gamepad1); // custom class which has useful button stuff like left_joystick_button_down
        secondaryPad = new Controller(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");

        controller = new PIDController(p, i , d);

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");

        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");

        waitForStart();

        //robot.intake.close();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            mainPad.update();
            secondaryPad.update();

            angles = robot.imu.getRobotYawPitchRollAngles();

            double pitch = angles.getPitch(AngleUnit.DEGREES);
            double yaw = angles.getYaw(AngleUnit.DEGREES);
            double roll = angles.getRoll(AngleUnit.DEGREES);

            double y = -gamepad1.left_stick_y; // forward/backward
            double x = gamepad1.left_stick_x * 1.8298514558; // right/left
            double rx = gamepad1.right_stick_x; // clockwise/counterclockwise

            double pid = controller.calculate(pitch, target); // pid for tip correction
            pid = Math.abs(pitch) < 3 || !doTipCorrection ? 0 : pid;

            double driveSpeed = !alignmentControl ? driveSpeedMode.speed : DriveConstants.ALIGNMENT_SPEED; // get the actual drive speed value

            // makes forward = current heading (only has affects in field centric)
            if (gamepad1.options) {
                // set the gyro parameters
                IMU.Parameters parameters = new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                LOGO_FACING_DIR,
                                USB_FACING_DIR
                        )
                );

                // initialize the gyro with these parameters
                robot.imu.initialize(parameters);
                robot.imu.resetYaw();
            }

            // go to the next drive speed mode
            if (mainPad.left_stick_down) {
                driveSpeedMode = driveSpeedMode.next();
            }

            // go to the previous drive speed mode
            if (mainPad.right_stick_down) {
                driveSpeedMode = driveSpeedMode.previous();
            }

            // if the dpad up was pressed switch driving modes (ex. switch to field centric if currently in robot centric)
            if (secondaryPad.dpad_up_pressed || mainPad.dpad_up_pressed) {
                robotCentric = !robotCentric;
            }

            // launch airplane when dpad down is pressed
            if (gamepad2.back || gamepad1.back) {
                robot.airplane.setPosition(1);
            }

            if (gamepad1.dpad_left) {
                robot.elbow.setPosition(125/180.0);
            }

            if (gamepad1.dpad_down) {
                robot.elbow.setPosition(55/180.0);
            }

            if (gamepad1.dpad_right) {
                robot.elbow.setPosition(0);
            }

            // if we are not reversed, open/close the right intake
            if (mainPad.right_bumper_down && !reverseDrive) {
                if (rightIntakeOpen) {
                    robot.rightIntake.setPosition(robot.intake.rightClosedPos);
                }
                else {
                    robot.rightIntake.setPosition(robot.intake.rightOpenPos);
                }
                rightIntakeOpen = !rightIntakeOpen;
            }
            // if we are reversed, open/close the left intake
            else if (reverseDrive && mainPad.right_bumper_down) {
                if (leftIntakeOpen) {
                    //robot.intake.closeLeft();
                    robot.leftIntake.setPosition(robot.intake.leftClosedPos);
                }
                else {
                    //robot.intake.openLeft();
                    robot.leftIntake.setPosition(robot.intake.leftOpenPos);
                }
                leftIntakeOpen = !leftIntakeOpen;
            }


            // don't do tip correction after pressing y (should be used for hanging)
            if (mainPad.y_down) {
                doTipCorrection = !doTipCorrection;
            }

//            // reverse the drive when back is clicked (this is meant for aligning with the backdrop)
//            if (mainPad.back_down) {
//                reverseDrive = !reverseDrive;
//            }

            // move the elbow up if x is pressed
            double deltaTime = (System.nanoTime() * 1e-9) - previousTime;

            // both controllers can control the elbow (2snd is supposed to though)
            if (gamepad1.x) {
                robot.elbow.changePosition(DriveConstants.ELBOW_SPEED * 3/4 * deltaTime);
                //telemetry.addData("Elbow Position", (1 - robot.elbow.leftPos) * 180);
            }
            if (gamepad1.b) {
                robot.elbow.changePosition(-DriveConstants.ELBOW_SPEED * 3/4 * deltaTime);
                //telemetry.addData("Elbow Position", (1 - robot.elbow.leftPos) * 180);
            }

            if (gamepad2.right_trigger > 0) {
                robot.elbow.changePosition(gamepad2.right_trigger * DriveConstants.ELBOW_SPEED * deltaTime);
            }
            if (gamepad2.left_trigger > 0) {
                robot.elbow.changePosition(-gamepad2.left_trigger * DriveConstants.ELBOW_SPEED * deltaTime);
            }

            // if we are not reversed, open/close the left intake
            if (mainPad.left_bumper_down && !reverseDrive) {
                if (leftIntakeOpen) {
                    //robot.intake.closeLeft();
                    robot.leftIntake.setPosition(robot.intake.leftClosedPos);
                }
                else {
                    //robot.intake.openLeft();
                    robot.leftIntake.setPosition(robot.intake.leftOpenPos);
                }
                leftIntakeOpen = !leftIntakeOpen;
            }
            // if we are reversed, open/close the right intake
            else if (mainPad.left_bumper_down) {
                if (rightIntakeOpen) {
                    robot.rightIntake.setPosition(robot.intake.rightClosedPos);
                }
                else {
                    robot.rightIntake.setPosition(robot.intake.rightOpenPos);
                }
                rightIntakeOpen = !rightIntakeOpen;
            }

            // Lift control
            if (gamepad1.right_trigger != 0) {
                robot.lift.setPower(liftSpeed * gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger != 0) {
                robot.lift.setPower(-liftSpeed * gamepad1.left_trigger);
            }
            else {
                robot.lift.setPower(0);
            }

            // Field Centric movement
            if (!robotCentric) {
                double robotHeading = angles.getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-robotHeading) - y * Math.sin(-robotHeading);
                double rotY = x * Math.sin(-robotHeading) + y * Math.cos(-robotHeading);

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                robot.frontLeft.setPower(frontLeftPower * driveSpeed - pid);
                robot.frontRight.setPower(frontRightPower * driveSpeed - pid);
                robot.backLeft.setPower(backLeftPower * driveSpeed + pid);
                robot.backRight.setPower(backRightPower * driveSpeed + pid);
            }
            // Robot Centric movement
            else {
                if (reverseDrive) {
                    x *= -1;
                    y *= -1;
                }

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                robot.frontLeft.setPower(frontLeftPower * driveSpeed - pid);
                robot.frontRight.setPower(frontRightPower * driveSpeed - pid);
                robot.backLeft.setPower(backLeftPower * driveSpeed + pid);
                robot.backRight.setPower(backRightPower * driveSpeed + pid);
            }

            // telemetry
            String driveMode = robotCentric ? "Robot Centric" : "Field Centric";
            String speedMode = driveSpeedMode.type;

            telemetry.addData("Drive Mode", driveMode);
            telemetry.addData("Speed Mode", speedMode);
            telemetry.addData("---------------------------------------------", "");

            String rightIntakeState = rightIntakeOpen ? "Open" : "Closed";
            String leftIntakeState = leftIntakeOpen ? "Open" : "Closed";

            telemetry.addData("Right Intake", rightIntakeState);
            telemetry.addData("Left Intake", leftIntakeState);

            telemetry.addData("Elbow Position", Math.round((1 - robot.elbow.leftPos) * 180));
            telemetry.addData("---------------------------------------------", "");

            telemetry.update();

            previousTime = System.nanoTime() * 1e-9;
        }

        robot.airplane.setPosition(0);
    }
}

