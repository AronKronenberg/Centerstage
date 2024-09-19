package org.firstinspires.ftc.teamcode;

public class Elbow {
    Hardware robot;

    public double leftPos;

    public Elbow(Hardware robot) {
        this.robot = robot;
        leftPos = 1;
    }

    public void changePosition(double position) {
        robot.leftElbow.setPosition(leftPos - position);

        leftPos = Math.max(Math.min(leftPos - position, 1), 0);
    }

    public void setPosition(double position) {
        robot.leftElbow.setPosition(1 - position);

        leftPos = Math.max(Math.min(1 - position, 1), 0);
    }
}
