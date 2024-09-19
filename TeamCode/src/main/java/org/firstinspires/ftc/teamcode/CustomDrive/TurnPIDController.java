package org.firstinspires.ftc.teamcode.CustomDrive;

import com.qualcomm.robotcore.util.ElapsedTime;

// PID Controller made for turning with degrees
public class TurnPIDController {
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = 0;

    private double targetAngle;
    private double kP, kI, kD;
    private double accumulatedError = 0;

    public TurnPIDController(double target, double p, double i, double d) {
        targetAngle = target;
        kP = p;
        kI = i;
        kD = d;
    }

    public double update(double currentAngle) {
        // P
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) {
            error -= 360;
        }

        // I
        accumulatedError += error;
        if (Math.abs(error) < 1) {
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error); // Make sure the accumulated error is working in the right direction

        // D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        // motor power calculation
        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(kP * error + kI * accumulatedError + kD * slope);
        return motorPower;
    }

}
