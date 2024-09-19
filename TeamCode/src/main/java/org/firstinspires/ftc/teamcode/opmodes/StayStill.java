package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Stay Still")
public class StayStill extends LinearOpMode {
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(1e-6)
                .build();
        Trajectory backward = drive.trajectoryBuilder(forward.end())
                .back(1e-6)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            drive.followTrajectory(forward);
            drive.followTrajectory(backward);
        }
    }
}
