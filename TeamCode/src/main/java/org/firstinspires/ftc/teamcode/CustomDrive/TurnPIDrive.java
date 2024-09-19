package org.firstinspires.ftc.teamcode.CustomDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Holds functions for PID gyro turning
public class TurnPIDrive extends Hardware {

    public TurnPIDrive(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap, opMode);
    }

    public void turnTo(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003); // TODO: tune the PID values for turning
        while (op.opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1) {
            double motorPower = pid.update(getAbsoluteAngle());
            setPower(-motorPower, motorPower, -motorPower, motorPower);
        }
        setPower(0);
    }

    public void turn(double degrees) {
        turnTo(degrees + getAbsoluteAngle());
    }
}
