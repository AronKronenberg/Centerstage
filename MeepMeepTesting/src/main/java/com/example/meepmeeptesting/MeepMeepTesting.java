package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))
                                .forward(24)
                                .turn(Math.toRadians(45))
                                .waitSeconds(2)
                                .turn(Math.toRadians(-45))
                                .back(12)
                                .splineTo(new Vector2d(47.5, 60), Math.toRadians(0))
                                .strafeLeft(18)
                                .waitSeconds(2)
                                .strafeRight(18)
                                .back(12)
                                .build()
//                                .lineToLinearHeading(new Pose2d(20, 34, Math.toRadians(-90)))
//                                .waitSeconds(2)
//                                .lineToLinearHeading(new Pose2d(25, 46, Math.toRadians(-45)))
//                                .splineToLinearHeading(new Pose2d(48 ,39), Math.toRadians(0))
//                                .lineToLinearHeading(new Pose2d(48, 62, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(62, 62, Math.toRadians(180)))
//                                .build()
                );

        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-58.5, 12, 0))
                                .forward(20)
                                .waitSeconds(4)
                                .back(10)
                                .splineToSplineHeading(new Pose2d(-35, 45, Math.toRadians(270)), Math.toRadians(50))
                                .waitSeconds(4)
                                .strafeRight(27)
                                .back(5)
                                .build()
                );
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(58.5, 12, Math.toRadians(180)))
                                .forward(20)
                                .waitSeconds(4)
                                .back(10)
                                .splineToSplineHeading(new Pose2d(35, 45, Math.toRadians(270)), Math.toRadians(130))
                                .waitSeconds(4)
                                .strafeLeft(25)
                                .back(5)
                                .build()
                );

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-58.5, -34.5, 0))
                                .forward(20)
                                .waitSeconds(4)
                                .back(20)
                                .waitSeconds(6)
                                .strafeLeft(25)
                                .splineToSplineHeading(new Pose2d(-35, 39.3, Math.toRadians(270)), Math.toRadians(50))
                                .back(5.7)
                                .build()
                );
        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(58.5, -34.5, Math.toRadians(180)))
                                .forward(20)
                                .waitSeconds(4)
                                .back(20)
                                .waitSeconds(6)
                                .strafeRight(25)
                                .splineToSplineHeading(new Pose2d(35, 39.3, Math.toRadians(270)), Math.toRadians(130))
                                .back(5.7)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.75f)
                .addEntity(blueLeft)
//                .addEntity(myBot1)
//                .addEntity(myBot2)
//                .addEntity(myBot3)
//                .addEntity(myBot4)
                .start();
    }
}