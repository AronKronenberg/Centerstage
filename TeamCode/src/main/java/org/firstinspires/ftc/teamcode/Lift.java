package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriveConstants.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.DriveConstants.LIFT_ANGLE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {
    LinearOpMode op;

    Hardware robot;

    public Lift(LinearOpMode op, Hardware robot) {
        this.op = op;
        this.robot = robot;
    }

    public void setPower(double power) {
        robot.liftLeft.setPower(power);
        robot.liftRight.setPower(power);
    }

    public void setMode(DcMotor.RunMode mode) {
        robot.liftLeft.setMode(mode);
        robot.liftRight.setMode(mode);
    }

    // function to set the height of the lift relative to the floor
    // ex. height = 15 would move the top of the lift 15 inches above the ground
    public void setHeight(double height) {
        // TODO: change retracted lift height to be the height of the lift at its lowest position
        double length = Math.max((height - DriveConstants.RETRACTED_LIFT_HEIGHT), 0) / Math.sin(Math.toRadians(LIFT_ANGLE));
        int newTarget = (robot.liftLeft.getCurrentPosition() + robot.liftRight.getCurrentPosition()) / 2 + (int) (length * COUNTS_PER_INCH);

        robot.liftLeft.setTargetPosition(newTarget);
        robot.liftRight.setTargetPosition(newTarget);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (op.opModeIsActive() && (robot.liftLeft.isBusy() && robot.liftRight.isBusy())) {
            // wait for the motors to be done reaching their position
        }

        // Stop all motion
        setPower(0);

        // Turn off RUN_TO_POSITION
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // function to set the height of the lift relative to the height at the lowest lift position
    // ex. height = 2 would move the top of the lift 2 inches vertically
    public void setHeight(double height, boolean relativeToLift) {
        double length;

        if (relativeToLift) length = height / Math.sin(Math.toRadians(LIFT_ANGLE));
        else length = Math.max((height - DriveConstants.RETRACTED_LIFT_HEIGHT), 0) / Math.sin(Math.toRadians((LIFT_ANGLE)));

        int newTarget = (robot.liftLeft.getCurrentPosition() + robot.liftRight.getCurrentPosition()) / 2 + (int) (length * COUNTS_PER_INCH);

        robot.liftRight.setTargetPosition(newTarget);
        robot.liftRight.setTargetPosition(newTarget);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (op.opModeIsActive() && (robot.liftLeft.isBusy() && robot.liftRight.isBusy())) {
            // wait for the motors to be done reaching their position
        }

        // Stop all motion
        setPower(0);

        // Turn off RUN_TO_POSITION
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
