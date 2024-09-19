package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PropDetectorProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Autonomous", group = "Autonomous", preselectTeleOp = "TeleOp")
public class SAR2D2Autonomous extends LinearOpMode {
    private PropDetectorProcessor propDetector;
    private VisionPortal visionPortal;

    private SampleMecanumDrive drive;

    private Hardware robot = new Hardware(); // need access to hardware class to perform lift and intake operations

    public enum StartPosition {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public StartPosition startPosition;

    public static PropDetectorProcessor.Location spikeMarkLocation = PropDetectorProcessor.Location.NOT_FOUND;

    double waitSecondsBeforeDrop = 2; // time to wait before heading to the backboard

    @Override
    public void runOpMode() throws InterruptedException {
        // Driver selects starting positions using controller1
        selectStartingPosition();

        telemetry.addData("Selected Starting Position", startPosition);
        telemetry.addData("Selected Waiting Time before going to the backdrop", waitSecondsBeforeDrop);

        // Initialize the robot for lift use
        robot.init(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize prop detecting processor and vision portal
        // Tell propDetector if we are red are blue so it can look for the right color
        propDetector = new PropDetectorProcessor(telemetry, startPosition == StartPosition.RED_LEFT || startPosition == StartPosition.RED_RIGHT);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), propDetector);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Start Position", startPosition);
            telemetry.addData("Selected Wait Time", waitSecondsBeforeDrop);

            // Get the prop location detected by the prop detector processor
            spikeMarkLocation = propDetector.getLocation();
            telemetry.addData("Prop Location", spikeMarkLocation);
            telemetry.update();
        }

        // Game Play Button is pressed
        if (opModeIsActive() && !isStopRequested()) {
            // Build parking trajectory based on last detected prop location
            if (spikeMarkLocation == PropDetectorProcessor.Location.NOT_FOUND) spikeMarkLocation = PropDetectorProcessor.Location.LEFT;
            runAutonomousMode();
        }
    }

    public void runAutonomousMode() {
        // Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(1e-7, 0, 0);
        Pose2d dropPurplePixelPose = new Pose2d(1e-7, 0, 0);
        Pose2d midwayPose1 = new Pose2d(1e-7,0,0);
        Pose2d midwayPose1a = new Pose2d(1e-7,0,0);
        Pose2d intakeStack = new Pose2d(1e-7,0,0);
        Pose2d midwayPose2 = new Pose2d(1e-7,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(1e-7, 0, 0);
        Pose2d parkPose = new Pose2d(1e-7, 0, 0);

        // Ignore for now as the selectWaitTime function should be called on init
        // TODO: Adjust time to wait for alliance partner to move from board (This is done in the selectWaitTime function)

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0));

        switch (startPosition) {
            case BLUE_LEFT:
                drive = new SampleMecanumDrive(hardwareMap);
                drive.setPoseEstimate(initPose);
                switch (spikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(23, 36, Math.toRadians(-90));
                        break;
                    case CENTER:
                        dropPurplePixelPose = new Pose2d(30, 3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(37, 36, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(30, -9, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(37, 36, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(14, 13, Math.toRadians(-45));
                parkPose = new Pose2d(8, 30, Math.toRadians(-90));
                break;

            case RED_RIGHT:
                drive = new SampleMecanumDrive(hardwareMap);
                drive.setPoseEstimate(initPose);
                switch (spikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(30, 9, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(21, -36, Math.toRadians(90));
                        break;
                    case CENTER:
                        dropPurplePixelPose = new Pose2d(30, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(29, -36, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(26, -8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(37, -36, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(14, -13, Math.toRadians(45));
                parkPose = new Pose2d(8, -30, Math.toRadians(90));
                break;

            case BLUE_RIGHT:
                drive = new SampleMecanumDrive(hardwareMap);
                drive.setPoseEstimate(initPose);
                switch (spikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(27, 9, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(27, 86, Math.toRadians(-90));
                        break;
                    case CENTER:
                        dropPurplePixelPose = new Pose2d(30, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(34, 86, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(26, -8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(43, 86, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(8, -8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, -18, Math.toRadians(-90));
                intakeStack = new Pose2d(52, -19,Math.toRadians(-90));
                midwayPose2 = new Pose2d(52, 62, Math.toRadians(-90));
                parkPose = new Pose2d(50, 84, Math.toRadians(-90));
                break;

            case RED_LEFT:
                drive = new SampleMecanumDrive(hardwareMap);
                drive.setPoseEstimate(initPose);
                switch (spikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(37, -86, Math.toRadians(90));
                        break;
                    case CENTER:
                        dropPurplePixelPose = new Pose2d(30, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(29, -86, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(27, -9, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(21, -86, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(8, 8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, 18, Math.toRadians(90));
                intakeStack = new Pose2d(52, 19,Math.toRadians(90));
                midwayPose2 = new Pose2d(52, -62, Math.toRadians(90));
                parkPose = new Pose2d(50, -84, Math.toRadians(90));
                break;
        }

        // Move robot to dropPurplePixel based on identified Spike Mark Location
        Trajectory dropPurplePixel = drive.trajectoryBuilder(initPose)
                .lineToLinearHeading(dropPurplePixelPose)
                .build();
        drive.followTrajectory(dropPurplePixel);

        // TODO: Add code to drop purple pixel on spike mark
        safeWaitSeconds(1);

        // Move robot to midwayPose1
        Trajectory moveToMidwayPose1 = drive.trajectoryBuilder(dropPurplePixel.end())
                .lineToLinearHeading(midwayPose1)
                .build();
        drive.followTrajectory(moveToMidwayPose1);

        safeWaitSeconds(waitSecondsBeforeDrop); // before going to the backdrop we wait some time so our teamate can go park

        // Move robot to midwayPoint2 and to dropYellowPixel
        Trajectory dropYellowPixel = drive.trajectoryBuilder(moveToMidwayPose1.end(), true)
                .splineToLinearHeading(dropYellowPixelPose, 0)
                .build();
        drive.followTrajectory(dropYellowPixel);

        // TODO: Add code to drop pixel on backdrop
        safeWaitSeconds(1);

        // Move robot to park in backstage
        Trajectory parkBackstage = drive.trajectoryBuilder(dropYellowPixel.end())
                .lineToLinearHeading(parkPose)
                .build();
        drive.followTrajectory(parkBackstage);
    }

    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();

        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");

            if (gamepad1.x) {
                startPosition = StartPosition.BLUE_LEFT;
                break;
            }
            if (gamepad1.y) {
                startPosition = StartPosition.BLUE_RIGHT;
                break;
            }
            if (gamepad1.b) {
                startPosition = StartPosition.RED_LEFT;
                break;
            }
            if (gamepad1.a) {
                startPosition = StartPosition.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    // method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}
