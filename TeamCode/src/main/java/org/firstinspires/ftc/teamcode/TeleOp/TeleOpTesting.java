package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="TeleOp Testing")
public class TeleOpTesting extends LinearOpMode {

    VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;

    Hardware robot = new Hardware();
    Controller mainPad;

    private PIDController controller;

    public static double p = 0.001, i = 0, d = 0;
    public static double f = -0.1;
    public static int target = 0;

    SampleMecanumDrive drive;

    boolean robotCentric = false;

    YawPitchRollAngles angles;

    Hardware.DriveSpeed driveSpeedMode = Hardware.DriveSpeed.NORMAL;

    double liftSpeed = Hardware.LIFT_SPEED; // current lift speed of the robot

    // This is for aligning with the april tags on the backdrop
    private static final int DESIRED_TAG_ID = -1; // Choose the tag you want to approach or set to -1 for ANY tag.

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize camera stuff (vision portal, april tag processor)
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);

        robot.init(this);
        drive = new SampleMecanumDrive(hardwareMap);
        mainPad = new Controller(gamepad1); // custom class which has useful button stuff like left_joystick_button_down

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");

        controller = new PIDController(p, i , d);

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Tell the driver that initialization is complete
        telemetry.addData("Status", "Initialized");

        waitForStart();

        robot.intake.close();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            mainPad.update();

            angles = robot.imu.getRobotYawPitchRollAngles();

            double robotPitch = angles.getPitch(AngleUnit.DEGREES);
            double robotYaw = angles.getYaw(AngleUnit.DEGREES);
            double robotRoll = angles.getRoll(AngleUnit.DEGREES);

            double y = -gamepad1.left_stick_y; // forward/backward
            double x = gamepad1.left_stick_x * 1.1; // right/left
            double rx = gamepad1.right_stick_x; // clockwise/counterclockwise

            double pid = controller.calculate(robotPitch, target); // pid for tip correction
            pid = Math.abs(robotPitch) < 6 ? 0 : pid;

            double driveSpeed = driveSpeedMode.speed; // get the actual drive speed value

            // makes forward = current heading (only has affects in field centric)
            if (gamepad1.options) {
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
            if (mainPad.dpad_up_pressed) {
                robotCentric = !robotCentric;
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

            // Aligning with april tag
            // If A is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.a && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DriveConstants.DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                x = Range.clip(rangeError * DriveConstants.SPEED_GAIN, -DriveConstants.MAX_AUTO_SPEED, DriveConstants.MAX_AUTO_SPEED);
                y = Range.clip(headingError * DriveConstants.TURN_GAIN, -DriveConstants.MAX_AUTO_TURN, DriveConstants.MAX_AUTO_TURN) ;
                rx = Range.clip(-yawError * DriveConstants.STRAFE_GAIN, -DriveConstants.MAX_AUTO_STRAFE, DriveConstants.MAX_AUTO_STRAFE);

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                robot.frontLeft.setPower(frontLeftPower);
                robot.frontRight.setPower(frontRightPower);
                robot.backLeft.setPower(backLeftPower);
                robot.backRight.setPower(backRightPower);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            // Field Centric movement
            else if (!robotCentric) {
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

            // for april tag alignment
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            String driveMode = robotCentric ? "Robot Centric" : "Field Centric";
            String speedMode = driveSpeedMode.type;

            telemetry.addData("Drive Mode", driveMode);
            telemetry.addData("Speed Mode", speedMode);

            telemetry.addData("Yaw", robotYaw);
            telemetry.addData("Pitch", robotPitch);
            telemetry.addData("Roll", robotRoll);

            telemetry.addData("target ", target); // for tip correction

            telemetry.update();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}

