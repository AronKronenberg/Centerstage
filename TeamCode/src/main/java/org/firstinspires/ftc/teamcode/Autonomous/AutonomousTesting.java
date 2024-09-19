package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

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

@Autonomous(name="Autonomous Testing")
public class AutonomousTesting extends LinearOpMode {
    SampleMecanumDrive drive;
    Hardware robot = new Hardware();

    PropDetectorProcessor propDetector;
    VisionPortal visionPortal;

    public enum StartPosition {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public SAR2D2Autonomous.StartPosition startPosition;

    public static PropDetectorProcessor.Location spikeMarkLocation = PropDetectorProcessor.Location.RIGHT;

    public void runOpMode() throws InterruptedException {
        // Driver selects starting positions using controller1
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        drive = new SampleMecanumDrive(hardwareMap);
        robot.init(this);

//        propDetector = new PropDetectorProcessor(startPosition == SAR2D2Autonomous.StartPosition.RED_LEFT || startPosition == SAR2D2Autonomous.StartPosition.RED_RIGHT);
//        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), propDetector);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        safeWaitSeconds(0.5); // wait just to make sure the prop detector has time to detect

        // spikeMarkLocation = propDetector.getLocation();
        telemetry.addData("Prop Location", spikeMarkLocation);
        telemetry.update();

        Pose2d initPose = new Pose2d(-3, 0, 0);
        drive.setPoseEstimate(initPose);

        double spikeMarkTurnAngle = 0; // default spike mark location is center

        switch (spikeMarkLocation) {
            case LEFT:
                spikeMarkTurnAngle = 90;
                break;

            case RIGHT:
                spikeMarkTurnAngle = -90;
                break;
            }

        switch (startPosition) {
            case BLUE_RIGHT:
                Trajectory moveToStart = drive.trajectoryBuilder(initPose)
                        .splineTo(new Vector2d(0, 0), 0)
                        .build();

                Trajectory moveTowardsSpikeMarks = drive.trajectoryBuilder(moveToStart.end())
                        .forward(24)
                        .build();

                Trajectory moveBackwardToStartPoint = drive.trajectoryBuilder(moveTowardsSpikeMarks.end().plus(new Pose2d(0, 0, Math.toRadians(spikeMarkTurnAngle))))
                        .lineTo(new Vector2d(0, 0))
                        .build();

                Trajectory moveTowardsBoard = drive.trajectoryBuilder(moveBackwardToStartPoint.end().plus(new Pose2d(0, 0, Math.toRadians(90 - spikeMarkTurnAngle))))
                        .forward(72)
                        .build();

                Trajectory allignWithBoard = drive.trajectoryBuilder(moveTowardsBoard.end())
                        .strafeRight(24)
                        .build();

                Trajectory allignWithParkLeft = drive.trajectoryBuilder(allignWithBoard.end())
                        .strafeLeft(24)
                        .build();

                Trajectory parkLeft = drive.trajectoryBuilder(allignWithParkLeft.end())
                        .forward(24)
                        .build();


                drive.followTrajectory(moveToStart);
                drive.followTrajectory(moveTowardsSpikeMarks);
                drive.turn(Math.toRadians(spikeMarkTurnAngle));
                drive.followTrajectory(moveBackwardToStartPoint);
                drive.turn(Math.toRadians(90 - spikeMarkTurnAngle));
                drive.followTrajectory(moveTowardsBoard);
                drive.followTrajectory(allignWithBoard);
                drive.followTrajectory(allignWithParkLeft);
                drive.followTrajectory(parkLeft);
        }
    }

    private void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();

        while (!isStopRequested()) {
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");

            if (gamepad1.x) {
                startPosition = SAR2D2Autonomous.StartPosition.BLUE_LEFT;
                break;
            }
            if (gamepad1.y) {
                startPosition = SAR2D2Autonomous.StartPosition.BLUE_RIGHT;
                break;
            }
            if (gamepad1.b) {
                startPosition = SAR2D2Autonomous.StartPosition.RED_LEFT;
                break;
            }
            if (gamepad1.a) {
                startPosition = SAR2D2Autonomous.StartPosition.RED_RIGHT;
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
