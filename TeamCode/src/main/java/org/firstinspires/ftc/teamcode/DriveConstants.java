package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class DriveConstants {
    // for aligning with an april tag
    // Adjust these numbers to suit your robot.
    public static final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public static final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    // the height the lift extends off the ground at its lowest point
    public static final double RETRACTED_LIFT_HEIGHT = 12.125;

    // for drive speed
    public static final double FAST_SPEED = 1;
    public static final double NORMAL_SPEED = 0.75;
    public static final double SLOW_SPEED = 0.375;

    public static final double ALIGNMENT_SPEED = 0.25;

    // for gyro
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    // for lift
    public static final double LIFT_SPEED = 0.75;

    public static final double LIFT_ANGLE = 33; // lift is mounted at 33 degrees
    public static final double INCHES_PER_ROTATION = 4.72441; // When the lift motor spins once, the lift moves this amount
    public static final double COUNTS_PER_MOTOR_REV = 537.7; // This amount of ticks = one revolution of the lift motor
    public static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / INCHES_PER_ROTATION; // every inch the lift travels is this amount of ticks of the motor

    // for elbow
    private static final double ELBOW_DEGREES_PER_SEC = 80;
    public static final double ELBOW_SPEED = ELBOW_DEGREES_PER_SEC / 180;
}
