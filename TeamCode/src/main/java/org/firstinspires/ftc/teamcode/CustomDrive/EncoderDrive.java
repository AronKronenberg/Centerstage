package org.firstinspires.ftc.teamcode.CustomDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Holds functions for encoder drive
public class EncoderDrive extends Hardware {

    public EncoderDrive(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap, opMode);
    }

    // positive = forward
    public void drive(double speed, double inches) {
        encoderDrive(speed, inches, inches, inches, inches);
    }

    // positive = counter-clockwise
    public void turn(double speed, double inches) {
        encoderDrive(speed, -inches, inches, -inches, inches);
    }

    // positive = strafe right
    public void strafe(double speed, double inches) {
        encoderDrive(speed, inches, -inches, -inches, inches);
    }

    // encoder drive method
    private void encoderDrive(double speed,
                              double frontLeft, double frontRight, double backLeft, double backRight) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the op mode is still active
        if (op.opModeIsActive()) {
            // Determine new target positions
            newLFTarget = getTargetPosition(this.frontLeft, frontLeft);
            newRFTarget = getTargetPosition(this.frontRight, frontRight);
            newLBTarget = getTargetPosition(this.backLeft, backLeft);
            newRBTarget = getTargetPosition(this.backRight, backRight);

            // pass target positions to motor controllers
            this.frontLeft.setTargetPosition(newLFTarget);
            this.frontRight.setTargetPosition(newRFTarget);
            this.backLeft.setTargetPosition(newLBTarget);
            this.backRight.setTargetPosition(newRBTarget);

            setDriveTrainMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // start motion
            setPower(Math.abs(speed));

            // Keep looping while we are still active and we haven't reached the target position
            while (op.opModeIsActive() &&
                    (this.frontLeft.isBusy() && this.frontRight.isBusy()
                            && this.backLeft.isBusy() && this.backRight.isBusy())) {
                // Don't do anything
                // this loop just makes sure the robot doesn't do anything else until it reaches the desired position

                // tells the driver the inches each motor has moved
                op.telemetry.addData("Front Left : ", toInches(this.frontLeft.getCurrentPosition()) + " inches");
                op.telemetry.addData("Front Right : ", toInches(this.frontRight.getCurrentPosition()) + " inches");
                op.telemetry.addData("Back Left : ", toInches(this.backLeft.getCurrentPosition()) + " inches");
                op.telemetry.addData("Back Right : ", toInches(this.backRight.getCurrentPosition()) + " inches");
                op.telemetry.update();
            }

            // stop all motion
            setPower(0);

            // Turn off RUN_TO_POSITION
            setDriveTrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
