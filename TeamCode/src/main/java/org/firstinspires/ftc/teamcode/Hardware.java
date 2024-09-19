package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.nio.charset.CharacterCodingException;

public class Hardware extends DriveConstants {
    HardwareMap hwMap; // hardware map reference

    public Lift lift; // lift object that has functions to control the lift
    public Intake intake; // intake object that has functions to control each/both intakes
    public Elbow elbow; // elbow object that has functions to control the elbow servos

    // enum for the different drive speeds
    public enum DriveSpeed {

        FAST(FAST_SPEED, "Fast"),
        NORMAL(NORMAL_SPEED, "Normal"),
        SLOW(SLOW_SPEED, "Slow");

        public final double speed;
        public final String type;

        DriveSpeed(double speed, String type) {
            this.speed = speed;
            this.type = type;
        }

        // get the next drive speed
        public DriveSpeed next() {
            int nextIndex = (this.ordinal() + 1) % values().length;
            return values()[nextIndex];
        }

        // get the previous drive speed
        public DriveSpeed previous() {
            int previousIndex = (this.ordinal() - 1 + values().length) % values().length;
            return values()[previousIndex];
        }
    }

    // Drive motors
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    // Lift motors
    public DcMotor liftLeft;
    public DcMotor liftRight;

    // Intake servos
    public Servo leftIntake;
    public Servo rightIntake;

    // Elbow servos
    public Servo leftElbow;
    public Servo rightElbow;

    // Airplane servo
    public Servo airplane;

    // Gyro
    public IMU imu;

    /* Constructor */
    public Hardware() {

    }

    // initialization method (needs a LinearOpMode just to check if opModeIsActive in Lift class)
    public void init(LinearOpMode opMode) {
        // save reference to Hardware map
        hwMap = opMode.hardwareMap;

        // initialize the drivetrain
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");

        // reverse the left-side drive motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // initialize the lift motors
        liftLeft = hwMap.get(DcMotor.class, "liftLeft");
        liftRight = hwMap.get(DcMotor.class, "liftRight");

        // reverse the left lift motor
        //liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.REVERSE);

        // initialize intake servos
        leftIntake = hwMap.get(Servo.class, "leftIntake");
        rightIntake = hwMap.get(Servo.class, "rightIntake");

        // initialize elbow servos
        leftElbow = hwMap.get(Servo.class, "leftElbow");
        rightElbow = hwMap.get(Servo.class, "rightElbow");

        // initialize airplane servo
        airplane = hwMap.get(Servo.class, "airplane");

        // initialize the gyro
        imu = hwMap.get(IMU.class, "imu");

        // set the gyro parameters
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        LOGO_FACING_DIR,
                        USB_FACING_DIR
                )
        );

        // initialize the gyro with these parameters
        imu.initialize(parameters);

        // initialize the lift object (not the motors)
        lift = new Lift(opMode, this);
        intake = new Intake(opMode, this);
        elbow = new Elbow(this);
    }

    // Method to set all the drive motor powers to a specific value
    public void setMotorPowers(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    // Method to set each individual motor power
    public void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft.setPower(frontLeft);
        this.frontRight.setPower(frontRight);
        this.backLeft.setPower(backLeft);
        this.backRight.setPower(backRight);
    }

    // Method to set the modes of every motor to a specific value
    public void setDriveTrainMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}

