package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.SparkyTheRobot;
import org.firstinspires.ftc.teamcode.Utilities.Control.StickyGamepad;

/*
 * Simple test of motion-profiled arm autonomous operation. The arm should move *smoothly*
 * between random angles.
 */
@Autonomous
@Config
public class ArmTestOpMode extends LinearOpMode {
    public static double factor = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        SparkyTheRobot robot = new SparkyTheRobot(this);
        Arm arm = new Arm(robot.leftFlipper, robot.rightFlipper, robot.linearSlide, robot.armIMU);
        StickyGamepad gp1 = new StickyGamepad(gamepad1);
        NanoClock clock = NanoClock.system();

        waitForStart();

        if (isStopRequested()) return;

        boolean prevUp = false;
        boolean prevDown = false;

        while (!isStopRequested()) {
            if (gamepad1.dpad_up && !prevUp) {
                prevUp = true;
                arm.setAngle(Arm.MAX_ANGLE * 0.5);
            }
            if (gamepad1.dpad_down && !prevDown) {
                prevDown = true;
                arm.setAngle(Arm.MAX_ANGLE * 1.0);
            }
            if (prevDown && !gamepad1.dpad_down) {
                prevDown = false;
            }
            if (prevUp && !gamepad1.dpad_up) {
                prevUp = false;
            }

            if (Math.abs(gamepad1.right_trigger - gamepad1.left_trigger) > 0.05) {
                arm.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            } else {
                arm.update();
            }
            Acceleration gravity = robot.armIMU.getGravity();
            telemetry.addData("Gravity", -Math.atan2(gravity.zAccel, gravity.yAccel));
            telemetry.addData("Gyro", robot.armIMU.getAngularOrientation().firstAngle);
            telemetry.update();
        }
    }
}
