package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.ConstrainedPIDMotor.Direction.BACKWARD;
import static org.firstinspires.ftc.teamcode.ConstrainedPIDMotor.Direction.COAST;
import static org.firstinspires.ftc.teamcode.ConstrainedPIDMotor.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.ConstrainedPIDMotor.Direction.HOLD;
import static org.firstinspires.ftc.teamcode.NullbotHardware.clamp;


@TeleOp(name="Nullbot: Teleop Nonrelative", group="Nullbot")
public class NullbotTeleopNonrelative extends LinearOpMode {

    NullbotHardware robot = new NullbotHardware();

    final double turnVolatility = 2; // Higher number makes turning more jerklike, but faster

    final double moveMotorThreshold = 0;
    final double triggerThreshold = 0.10;
    final double minSlowModePower = 0.15;
    final int headingLockMS = 1000;
    final int motorEncoderMS = 200;
    double initialHeading;
    double desiredHeading;
    double difference;
    double turnSpeed;
    double desiredMax;
    double heading;

    boolean wasLeftBumperPressed;
    boolean wasRightBumperPressed;
    boolean wasRightTriggerPressed;
    boolean nonrelativeDriveModeEnabled;
    boolean scale;

    ElapsedTime timeTillHeadingLock;

    ConstrainedPIDMotor lift;
    ConstrainedPIDMotor zType;

    ElapsedTime timeSinceDriveModeToggle;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this, true, gamepad2);

        lift = new ConstrainedPIDMotor(robot.lift, 100, 0.6, 0.6, 0, -1300);
        zType = new ConstrainedPIDMotor(robot.zType, 100, 0.2, 0.15, 0, 12288);

        waitForStart();

        initialHeading = robot.getGyroHeading();
        desiredHeading = initialHeading;

        wasLeftBumperPressed = false;
        wasRightBumperPressed = false;
        timeTillHeadingLock = new ElapsedTime();

        timeSinceDriveModeToggle = new ElapsedTime();
        nonrelativeDriveModeEnabled = true;
        wasRightTriggerPressed = false;

        robot.adjustMotorEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            // Toggle control mode
            if (gamepad1.right_trigger > triggerThreshold && !wasRightTriggerPressed &&
                    timeSinceDriveModeToggle.milliseconds() > 100) {
                nonrelativeDriveModeEnabled = !nonrelativeDriveModeEnabled;
                timeSinceDriveModeToggle.reset();
            }

            wasRightTriggerPressed = gamepad1.right_trigger > triggerThreshold;

            scale = true;
            // Calculate speed reduction
            desiredMax = 1;

            // Toggle motor encoders
            if (gamepad1.left_trigger > triggerThreshold) {// Left trigger activates slow mode
                desiredMax = minSlowModePower + ((1 - minSlowModePower) * (1 - gamepad1.left_trigger));
                robot.adjustMotorEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (!robot.isTestChassis) {

                setOverride(gamepad1.a);

                if (gamepad1.back) { // Back coasts all motors
                    lift.setDirection(COAST);
                    zType.setDirection(COAST);
                } else {
                    // Adjust lift
                    if (gamepad1.dpad_up) {
                        lift.setDirection(FORWARD);
                    } else if (gamepad1.dpad_down) {
                        lift.setDirection(BACKWARD);
                    } else {
                        lift.setDirection(HOLD);
                    }

                    if (gamepad1.dpad_right) {
                        zType.setDirection(FORWARD);
                    } else if (gamepad1.dpad_left) {
                        zType.setDirection(BACKWARD);
                    } else {
                        zType.setDirection(HOLD);
                    }
                    if (gamepad1.x) {
                        robot.closeBlockClaw();
                    } else if (gamepad1.b) {
                        robot.openBlockClaw();
                    }

                    if (!gamepad1.y) {
                        robot.raiseWhipSnake();
                    } else {
                        robot.lowerWhipSnake();
                    }
                }
            }

            boolean turnRelevant = Math.abs(gamepad1.right_stick_x) > 0.25;

            // Auto turning code
            heading = robot.getGyroHeading();


            if (turnRelevant || timeTillHeadingLock.milliseconds() < headingLockMS){
                desiredHeading = heading;
                turnSpeed = 0;

                if (turnRelevant) {
                    turnSpeed = gamepad1.right_stick_x;
                    if (gamepad1.left_trigger > triggerThreshold) {
                        turnSpeed /= 10.0;
                    }
                    timeTillHeadingLock.reset();
                }
            } else {
                // Auto turning
                if (true) {
                    difference = getAngleDifference(desiredHeading, heading);
                    turnSpeed = difference / (Math.PI / turnVolatility);
                    turnSpeed = clamp(turnSpeed);
                } else {
                    turnSpeed = 0;
                }
            }

            double[] unscaledMotorPowers = getDesiredDirection();
            // Add turning and scaling to powers

            for (int i = 0; i < unscaledMotorPowers.length; i++) {
                if (i % 2 == 0) {
                    unscaledMotorPowers[i] += turnSpeed;
                } else {
                    unscaledMotorPowers[i] -= turnSpeed;
                }
            }

            // Turning around a point bypasses all previous settings
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                robot.adjustMotorEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
                if (gamepad1.left_bumper) {
                    unscaledMotorPowers = new double[]{-1, -0.8, 1, 0.8};
                } else {
                    unscaledMotorPowers = new double[]{0.8, 1, -0.8, -1};
                }

            } else if (gamepad1.left_trigger < triggerThreshold) { // If there's nothing that needs precision running
                robot.adjustMotorEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (scale) {
                // Now, let's scale those powers
                double greatest = 0;
                for (double d : unscaledMotorPowers) {
                    greatest = Math.max(greatest, Math.abs(d));
                }
                greatest /= desiredMax;
                for (int i = 0; i < unscaledMotorPowers.length; i++) {
                    unscaledMotorPowers[i] = chop(unscaledMotorPowers[i] / greatest);
                }
            }

            robot.setMotorSpeeds(unscaledMotorPowers);

            telemetry.addLine()
                    .addData("Is X pressed", gamepad1.x);
            telemetry.update(); // Send telemetry data to driver station

            // Run above code at 25hz
            robot.writeLogTick(gamepad1);
            robot.waitForTick(1000 / robot.hz);
        }

    }
    private double chop(double d) { // Cutoff all signals being sent to the motor below a threshold
        if (Math.abs(d) < moveMotorThreshold) {
            return 0;
        } else {
            return d;
        }
    }

    public double getAngleDifference(double d1, double d2) {
        double diff = d2 - d1;
        if (d1 > Math.PI) {d1 -= 2 * Math.PI;}
        if (d2 > Math.PI) {d2 -= 2 * Math.PI;}

        double diff2 = d2 - d1;

        if (Math.abs(diff) < Math.abs(diff2)) {
            return diff;
        } else {
            return diff2;
        }
    }

    public double getLeftStickDist(Gamepad g) {
        return Math.sqrt(g.left_stick_x*g.left_stick_x + g.left_stick_y*g.left_stick_y);
    }

    public double[] getDesiredDirection() {
        double controllerAngle;
        if (getLeftStickDist(gamepad1) > triggerThreshold) {
            controllerAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI/2;
            controllerAngle = robot.normAngle(controllerAngle);
        } else {
            // If we're not moving, don't scale the values
            scale = false;
            return new double[]{0, 0, 0, 0};
        }

        double robotAngle;

        if (nonrelativeDriveModeEnabled) { // Right trigger activates nonrelative drive
            robotAngle = robot.normAngle(controllerAngle + heading);
        } else { // Default is relative drive
            robotAngle = controllerAngle;
        }

        double[] unscaledPowers = new double[4];
        unscaledPowers[0] = Math.sin(robotAngle + Math.PI/4);
        unscaledPowers[1] = Math.cos(robotAngle + Math.PI/4);
        unscaledPowers[2] = unscaledPowers[1];
        unscaledPowers[3] = unscaledPowers[0];
        return unscaledPowers;
    }

    public void setOverride (boolean b) {
        lift.override = b;
        zType.override = b;
    }
}
