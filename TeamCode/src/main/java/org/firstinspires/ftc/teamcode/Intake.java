package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Intake {
    Hardware robot;
    LinearOpMode op;

    // TODO: Assign the correct values to open and close each intake
    final double openPos = 0;
    final double closePos = 0.5;

    public final double leftOpenPos = 0.55;
    public final double leftClosedPos = 0.28;

    public final double rightOpenPos = 0.45;
    public final double rightClosedPos = 0.695;

    // Constructor
    public Intake(LinearOpMode op, Hardware robot) {
        this.op = op;
        this.robot = robot;
    }


    // Method to open the entire intake
    public void open() {
        robot.leftIntake.setPosition(leftOpenPos);
        robot.rightIntake.setPosition(rightOpenPos);
    }

    // Method to close the entire intake
    public void close() {
        robot.leftIntake.setPosition(leftClosedPos);
        robot.rightIntake.setPosition(rightClosedPos);
    }

    // Method to open the left intake
    public void openLeft() {
        robot.leftIntake.setPosition(openPos);
    }

    // Method to close the left intake
    public void closeLeft() {
        robot.leftIntake.setPosition(closePos);
    }

    // Method to open the right intake
    public void openRight() {
        robot.rightIntake.setPosition(openPos);
    }

    // Method to close the right intake
    public void closeRight() {
        robot.rightIntake.setPosition(closePos);
    }
}
