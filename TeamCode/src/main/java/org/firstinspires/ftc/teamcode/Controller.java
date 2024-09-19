package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    Gamepad gamepad;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean left_stick_down = false;
    public boolean right_stick_down = false;

    public boolean left_bumper_down = false;
    public boolean right_bumper_down = false;

    public boolean dpad_up_pressed = false;

    public boolean back_down = false;

    public boolean y_down = false;

    private boolean leftStickPressedLastFrame = false;
    private boolean rightStickPressedLastFrame = false;

    private boolean leftBumperPressedLastFrame = false;
    private boolean rightBumperPressedLastFrame = false;

    private boolean dPadUpPressedLastFrame = false;

    private boolean backPressedLastFrame = false;

    private boolean yDownLastFrame = false;

    public void update() {
        left_stick_down = gamepad.left_stick_button && !leftStickPressedLastFrame;
        right_stick_down = gamepad.right_stick_button && !rightStickPressedLastFrame;

        left_bumper_down = gamepad.left_bumper && !leftBumperPressedLastFrame;
        right_bumper_down = gamepad.right_bumper && !rightBumperPressedLastFrame;

        dpad_up_pressed = gamepad.dpad_up && !dPadUpPressedLastFrame;

        back_down = gamepad.back && !backPressedLastFrame;

        y_down = gamepad.y && !yDownLastFrame;

        leftStickPressedLastFrame = gamepad.left_stick_button;
        rightStickPressedLastFrame = gamepad.right_stick_button;

        leftBumperPressedLastFrame = gamepad.left_bumper;
        rightBumperPressedLastFrame = gamepad.right_bumper;

        dPadUpPressedLastFrame = gamepad.dpad_up;

        backPressedLastFrame = gamepad.back;

        yDownLastFrame = gamepad.y;
    }
}
