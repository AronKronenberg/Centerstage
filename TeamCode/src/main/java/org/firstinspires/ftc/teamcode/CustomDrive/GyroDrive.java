package org.firstinspires.ftc.teamcode.CustomDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Holds functions for basic gyro turning
public class GyroDrive extends Hardware {

    public GyroDrive(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap, opMode);
    }

    public void turn(double degrees) {
        resetAngle();

        double error = degrees;

        while (op.opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -TURN_SPEED : TURN_SPEED);
            setPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            op.telemetry.addData("turn error : ", error);
            op.telemetry.update();
        }

        setPower(0);
    }

    public void turnTo(double degrees) {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn(error);
    }
}
