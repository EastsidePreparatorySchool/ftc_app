package org.firstinspires.ftc.teamcode.RoverRuckus.Mappings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class SoloMapping extends ControlMapping {
    public static double INTAKE_SPEED = 0.85;
    public static double INTAKE_SLOW_SPEED = 0.2;

    public static double FLIP_LEFT_FACTOR = 0.65;
    public static double FLIP_RIGHT_FACTOR = 0.6;

    public static double MIN_SLOW_MOVE_SPEED = 0.4;

    //these should really be in mappings, not in teleop
    public static double TRANSLATE_POWER = 3; //the power to which joystick distance in exponented
    public static double TURN_POWER = 2.0;

    public int spinDir;
    private boolean x_down, b_down;

    public SoloMapping(Gamepad gamepad1, Gamepad gamepad2) {
        super(gamepad1, gamepad2);
        spinDir = 0;
    }

    @Override
    public double driveStickX() {
        return Math.pow(gamepad1.left_stick_x, TRANSLATE_POWER);
    }

    @Override
    public double driveStickY() {
        return Math.pow(gamepad1.left_stick_y, TRANSLATE_POWER);
    }

    @Override
    public double turnSpeed() {
        //cube values for bonus precision
        double sign = gamepad1.right_stick_x < 0 ? -1 : 1;
        double magnitude = gamepad1.right_stick_x * sign;

        return sign * Math.pow(magnitude, TURN_POWER);
    }

    @Override
    public double moveSpeedScale() {
        return stickyGamepad1.right_stick_button ? MIN_SLOW_MOVE_SPEED : 1;
    }

    final static double MIN_ARM_MOVE_SPEED = 0.15;
    @Override
    public double armSpeed() {
        return removeLowVals(gamepad1.right_trigger * FLIP_RIGHT_FACTOR - gamepad1.left_trigger * FLIP_LEFT_FACTOR, 0.05);
    }

    @Override
    public boolean flipOut() {
        return gamepad1.dpad_left;
    }

    @Override
    public boolean flipBack() {
        return gamepad1.dpad_right;
    }

    @Override
    public boolean shakeCamera() {
        return gamepad1.a;
    }

    @Override
    public double getSpinSpeed() {
        if (gamepad1.x && !x_down) {

            // X was just pressed
            spinDir = (spinDir == -1) ? 0 : -1;
            x_down = true;
        } else if (!gamepad1.x && x_down) {
            x_down = false;
        }

        if (gamepad1.b && !b_down) {
            // B was just pressed
            spinDir = (spinDir == 1) ? 0 : 1;
            b_down = true;
        } else if (!gamepad1.b && b_down) {
            b_down = false;
        }
        return spinDir * INTAKE_SPEED;
    }

    @Override
    public boolean override() {
        return gamepad1.start;
    }

    @Override
    public double getExtendSpeed() {
        return boolsToDir(gamepad1.dpad_down, gamepad1.dpad_up);
    }

    @Override
    public int getHangDir() {
        return boolsToDir(gamepad1.left_bumper, gamepad1.right_bumper);
    }
}
