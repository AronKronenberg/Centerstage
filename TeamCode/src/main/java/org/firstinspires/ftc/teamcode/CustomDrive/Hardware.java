package org.firstinspires.ftc.teamcode.CustomDrive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {
    HardwareMap hwMap;
    LinearOpMode op;

    // Encoder variables
    static final double COUNTS_PER_MOTOR_REV = 28; // Motor's encoder ticks per revolution
    static final double DRIVE_GEAR_REDUCTION = 1; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 2.953; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.3;

    // Gyro variables
    public Orientation lastAngle = new Orientation();
    public double currAngle = 0.0;

    // Hardware Objects (ex. motors, gyros, servos...)
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public BNO055IMU imu;

    // Controller classes
    public GyroDrive gyroDrive;
    public TurnPIDrive turnPIDrive;
    public EncoderDrive encoderDrive;

    public Hardware(HardwareMap hardwareMap, LinearOpMode opMode) {
        // Save reference to the hardware map and linear op mode
        hwMap = hardwareMap;
        op = opMode;

        // Define motors
        frontLeft = hwMap.get(DcMotorEx.class, "front left");
        frontRight = hwMap.get(DcMotorEx.class, "front right");
        backLeft = hwMap.get(DcMotorEx.class, "back left");
        backRight = hwMap.get(DcMotorEx.class, "back right");

        // Initialize motor direction
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Define the gyro
        imu = hwMap.get(BNO055IMU.class, "imu");

        // Set gyro parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the gyro
        imu.initialize(parameters);

        // Define all the different types of drives
        encoderDrive = new EncoderDrive(hwMap, op);
        gyroDrive = new GyroDrive(hwMap, op);
        turnPIDrive = new TurnPIDrive(hwMap, op);
    }


    // sets the power of all motors
    public void setPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    // sets the power of each individual motor
    public void setPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft.setPower(frontLeft);
        this.frontRight.setPower(frontRight);
        this.backLeft.setPower(backLeft);
        this.backRight.setPower(backRight);
    }

    // sets the mode of every motor (ex. RUN_USING_ENCODER)
    public void setDriveTrainMode(DcMotorEx.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }


    // converts the amount of encoder ticks of the motor to the inches it would cause a wheel to rotate
    double toInches(double ticks) {
        return ticks / COUNTS_PER_INCH;
    }

    // converts the amount of inches the wheel rotates to encoder ticks of the motor
    double toTicks(double inches) {
        return inches * COUNTS_PER_INCH;
    }

    // gets the new encoder target position for a motor given an amount of ticks to add
    // (ex. added target = 200, current encoder position = 400: target position = 600)
    int getTargetPosition(DcMotorEx motor, double target) {
        return motor.getCurrentPosition() + (int) toTicks(target);
    }


    // sets 0 degrees to the current robot angle
    void resetAngle() {
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    // gets the relative angle of the robot to zero
    double getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngle.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngle = orientation;
        op.telemetry.addData("gyro : ", orientation.firstAngle);
        return currAngle;
    }


    // gets the absolute angle of the robot relative to the field
    double getAbsoluteAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}

