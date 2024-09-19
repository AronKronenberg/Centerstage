package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;

@Config
@TeleOp(name="Servo Tester")
public class ServoTester extends LinearOpMode {
    public static double angle = 0.5;
    public static double originalAngle = 1;

    Servo servo;
    Servo servo2;

    Controller controller;

    boolean reversed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new Controller(gamepad1);

        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.addData("Initialized", "Click play");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            controller.update();

            double degrees = angle * 180;
            telemetry.addData("Change servo position to " + degrees + "°", "Y");
            telemetry.addData("Change servo position to 0°", "A");
            telemetry.addData("Change servo direction", "X");
            telemetry.addData("-------------------------------------", "");

            if (gamepad1.y) {
                servo.setPosition(1 - angle);
                servo2.setPosition(angle);
                telemetry.addData("Setting servo position to", degrees + "°");
            }
            if (gamepad1.a) {
                servo.setPosition(1);
                servo2.setPosition(0);
                telemetry.addData("Setting servo position to", "0°");
            }
            if (controller.right_stick_down) {
                if (reversed) {
                    servo.setDirection(Servo.Direction.FORWARD);
                    servo2.setDirection(Servo.Direction.FORWARD);
                    telemetry.addData("Changing servo direction to", "forward");
                }
                else {
                    servo.setDirection(Servo.Direction.REVERSE);
                    servo2.setDirection(Servo.Direction.REVERSE);
                    telemetry.addData("Changing servo direction to", "reverse");
                }
                reversed = !reversed;
            }

            telemetry.addData("Servo Direction", servo.getDirection());
            telemetry.update();
        }
    }
}
