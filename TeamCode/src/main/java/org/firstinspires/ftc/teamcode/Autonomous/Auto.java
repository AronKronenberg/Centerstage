package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PropDetectorProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Auto {

    LinearOpMode opmode;
    Hardware robot = new Hardware();
    SampleMecanumDrive drive;

    public enum StartPosition {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    public Auto(LinearOpMode opmode, Hardware robot, SampleMecanumDrive drive) {
        this.opmode = opmode;
        this.robot = robot;
        this.drive = drive;
    }

    public void runAutonomousMode(StartPosition startPosition, PropDetectorProcessor.Location spikeMarkLocation) {
        // TODO: Add code to grab the purple and yellow pixels

        // set values to move to place the purple pixel depending on where the prop is
        Pose2d purplePixelDropPose = new Pose2d(24, 0, Math.toRadians(0));

        // set values to move away from the truss depending on where the prop was placed
        Pose2d moveFromTruss = new Pose2d(24, 21, Math.toRadians(-90));

        // set values to move to the spot where we are going to travel to the backboard
        Pose2d backdropRendezvousPoint = new Pose2d(0, 0, Math.toRadians(-90));

        // set values to move to the correct spot on the backboard to place the yellow pixel
        Pose2d yellowPixelPlacement = new Pose2d(backdropRendezvousPoint.getX(), 24, Math.toRadians(-90));

        // set values to park after placing the yellow pixel
        Pose2d parkPosition = new Pose2d(45, -21, Math.toRadians(90));

        // value to drive up to board when placing the yellow pixel
        Pose2d placeOnBoard = new Pose2d(55.8, -26, Math.toRadians(-90));

        switch (startPosition) {
            case RED_LEFT:
                switch (spikeMarkLocation) {
                    case LEFT:
                        // if the prop is on the left, move a bit left to avoid truss and go forward
                        purplePixelDropPose = new Pose2d(22, 9.5, Math.toRadians(0));
                        moveFromTruss = new Pose2d(16, 18, Math.toRadians(-45));
                        backdropRendezvousPoint = new Pose2d(2, 0, Math.toRadians(-90));
                        yellowPixelPlacement = new Pose2d(31, -65, Math.toRadians(90));
                        parkPosition = new Pose2d(48.5, -73, Math.toRadians(90));
                        break;

                    case CENTER:
                        // if the prop is in the center move a bit left to avoid the truss and go forward
                        purplePixelDropPose = new Pose2d(25, 0, Math.toRadians(0));
                        moveFromTruss = new Pose2d(24, 14, Math.toRadians(-45));
                        backdropRendezvousPoint = new Pose2d(5, 0, Math.toRadians(-90));
                        yellowPixelPlacement = new Pose2d(31, -68, Math.toRadians(90));
                        parkPosition = new Pose2d(53.25, -73, Math.toRadians(90));
                        break;

                    case RIGHT:
                        // if the prop is on the right, go forward 1 tile and turn right
                        purplePixelDropPose = new Pose2d(24, -3, Math.toRadians(-45));
                        moveFromTruss = new Pose2d(24, 18, Math.toRadians(-90));
                        backdropRendezvousPoint = new Pose2d(4, 0, Math.toRadians(-90));
                        yellowPixelPlacement = new Pose2d(20, -66.5, Math.toRadians(-90));
                        parkPosition = new Pose2d(50, -73, Math.toRadians(-90));
                        break;
                }
                break;

            case RED_RIGHT:
                switch (spikeMarkLocation) {
                    case LEFT:
                        // if the prop is on the left, move a bit left to avoid truss and go forward
                        purplePixelDropPose = new Pose2d(32, -2.5, Math.toRadians(90));
                        moveFromTruss = new Pose2d(12, -19, Math.toRadians(-90));
                        yellowPixelPlacement = new Pose2d(30, -21, Math.toRadians(-90));
                        placeOnBoard = new Pose2d(34, -37, Math.toRadians(-90));
                        parkPosition = new Pose2d(51, -40, Math.toRadians(-90));
                        break;

                    case CENTER:
                        // if the prop is in the center, move a bit right
                        purplePixelDropPose = new Pose2d(27.5, 0, Math.toRadians(0));
                        moveFromTruss = new Pose2d(18, -19, Math.toRadians(-90));
                        yellowPixelPlacement = new Pose2d(29, -21, Math.toRadians(-90));
                        placeOnBoard = new Pose2d(32, -36.5, Math.toRadians(-90));
                        parkPosition = new Pose2d(53.5, -38, Math.toRadians(-90));
                        break;

                    case RIGHT:
                        purplePixelDropPose = new Pose2d(24, -3, Math.toRadians(-45));
                        moveFromTruss = new Pose2d(8, -24, Math.toRadians(-90));
                        yellowPixelPlacement = new Pose2d(26, -26, Math.toRadians(-90));
                        placeOnBoard = new Pose2d(26, -39, Math.toRadians(-90));
                        parkPosition = new Pose2d(53.5, -44, Math.toRadians(-90));
                        break;
                }
                break;

            case BLUE_LEFT:
                switch (spikeMarkLocation) {
                    case LEFT:
                        // if the prop is on the left, move a bit left to avoid truss and go forward
                        purplePixelDropPose = new Pose2d(34, -2.5, Math.toRadians(90));
                        moveFromTruss = new Pose2d(3, 0, Math.toRadians(90));
                        yellowPixelPlacement = new Pose2d(15, 21, Math.toRadians(90));
                        placeOnBoard = new Pose2d(19.5, 39, Math.toRadians(90));
                        parkPosition = new Pose2d(51, 41, Math.toRadians(90));
                        break;

                    case CENTER:
                        // if the prop is in the center, move a bit right
                        purplePixelDropPose = new Pose2d(26.5, 0, Math.toRadians(0));
                        moveFromTruss = new Pose2d(18, 19, Math.toRadians(90));
                        yellowPixelPlacement = new Pose2d(29, 21, Math.toRadians(90));
                        placeOnBoard = new Pose2d(25.5, 38, Math.toRadians(90));
                        parkPosition = new Pose2d(53.5, 38, Math.toRadians(90));
                        break;

                    case RIGHT:
                        purplePixelDropPose = new Pose2d(24, 1, Math.toRadians(-90));
                        moveFromTruss = new Pose2d(8, 24, Math.toRadians(90));
                        yellowPixelPlacement = new Pose2d(26, 26, Math.toRadians(90));
                        placeOnBoard = new Pose2d(29, 38.5, Math.toRadians(90));
                        parkPosition = new Pose2d(51.5, 44, Math.toRadians(90));
                        break;
                }
                break;

            case BLUE_RIGHT:
                switch (spikeMarkLocation) {
                    case LEFT:
                        purplePixelDropPose = new Pose2d(32, -2.5, Math.toRadians(90));
                        break;
                    case CENTER:
                        purplePixelDropPose = new Pose2d(24, 0, Math.toRadians(0));
                        break;
                    case RIGHT:
                        purplePixelDropPose = new Pose2d(24, 1, Math.toRadians(-90));
                        break;
                }
                break;
        }

        // robot is further from the backdrop
        if (startPosition == StartPosition.RED_LEFT || startPosition == StartPosition.BLUE_RIGHT) {
            // TODO: Add code to grab the purple and yellow pixels
            robot.intake.close();
            robot.elbow.setPosition(0.5);
            safeWaitSeconds(0.75);

            Trajectory moveToSpikeMark = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(purplePixelDropPose)
                    .build();
            drive.followTrajectory(moveToSpikeMark);

            // TODO: Add code to drop the purple pixel
            robot.rightIntake.setPosition(robot.intake.rightClosedPos + 0.1);
            robot.elbow.setPosition(0.1);
            safeWaitSeconds(0.4);
            robot.elbow.setPosition(0);
            safeWaitSeconds(0.25);
            robot.leftIntake.setPosition(robot.intake.leftOpenPos);
            safeWaitSeconds(0.25);

            // TODO: Pull up the elbow so it doesn't touch the floor
            robot.elbow.setPosition(0.1);
        }
        // robot is closer to the backdrop
        else {
            // TODO: Add code to grab the purple and yellow pixels
            robot.intake.close();
            robot.elbow.setPosition(0.5);
            safeWaitSeconds(0.75);

            Trajectory moveToSpikeMark = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(purplePixelDropPose)
                    .build();
            drive.followTrajectory(moveToSpikeMark);

            // TODO: Add code to drop the purple pixel
            robot.rightIntake.setPosition(robot.intake.rightClosedPos + 0.1);
            robot.elbow.setPosition(0.1);
            safeWaitSeconds(0.4);
            robot.elbow.setPosition(0);
            safeWaitSeconds(0.25);
            robot.leftIntake.setPosition(robot.intake.leftOpenPos);
            safeWaitSeconds(0.25);

            // TODO: Pull up the elbow so it doesn't touch the floor
            robot.elbow.setPosition(0.1);

            safeWaitSeconds(1);

            Trajectory moveFromTruss2 = drive.trajectoryBuilder(moveToSpikeMark.end())
                    .lineToLinearHeading(moveFromTruss)
                    .build();
            drive.followTrajectory(moveFromTruss2);

            safeWaitSeconds(1);

            Trajectory moveToYellowPixelDropLocation = drive.trajectoryBuilder(moveFromTruss2.end())
                    .lineToLinearHeading(yellowPixelPlacement)
                    .build();
            drive.followTrajectory(moveToYellowPixelDropLocation);

            // TODO: Add code to extend the elbow and drop the yellow pixel
            robot.elbow.setPosition(0.23);
            safeWaitSeconds(0.5);

            // drive up to the board to place the yellow pixel
            Trajectory moveToBoard = drive.trajectoryBuilder(moveToYellowPixelDropLocation.end())
                    .lineToLinearHeading(placeOnBoard)
                    .build();
            drive.followTrajectory(moveToBoard);

            // open the intake to place the yellow pixel
            robot.rightIntake.setPosition(robot.intake.rightOpenPos);

            // TODO: Add reset the intake and retract the lift

            safeWaitSeconds(1);

            Trajectory backFromBackdrop = drive.trajectoryBuilder(moveToBoard.end())
                    .lineToLinearHeading(new Pose2d(placeOnBoard.getX(), placeOnBoard.getY() + 4 * Math.signum(-placeOnBoard.getHeading()), placeOnBoard.getHeading()))
                    .build();
            drive.followTrajectory(backFromBackdrop);

            Trajectory alignToPark = drive.trajectoryBuilder(backFromBackdrop.end())
                    .lineToLinearHeading(new Pose2d(parkPosition.getX(), yellowPixelPlacement.getY(), parkPosition.getHeading()/*Math.toRadians(-90)*/))
                    .build();
            drive.followTrajectory(alignToPark);

            safeWaitSeconds(1);

            Trajectory park = drive.trajectoryBuilder(alignToPark.end())
                    .lineToLinearHeading(parkPosition)
                    .build();
            drive.followTrajectory(park);
        }
    }

    // method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!opmode.isStopRequested() && timer.time() < time) {
        }
    }
}
