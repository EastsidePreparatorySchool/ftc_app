package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Extender;
import org.firstinspires.ftc.teamcode.Mechanisms.SparkyTheRobot;
import org.firstinspires.ftc.teamcode.RoverRuckus.Auto.AutoUtils;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mappings.ControlMapping;
import org.firstinspires.ftc.teamcode.Utilities.Control.FeedbackController;
import org.firstinspires.ftc.teamcode.Utilities.Control.HoldingPIDMotor;
import org.firstinspires.ftc.teamcode.Utilities.Control.WheelDriveVector;

@Config
public abstract class BaseTeleOp extends LinearOpMode {

    // Where the robot will be facing when the "heading reset" button is clicked
    public static double HEADING_RESET_POSITION = Math.PI * 0.25;

    // How fast the robot moves when the arm is fully extended/retracted
    public static double EXTEND_MAXED_DRIVE_POWER = 0.6;

    // Constants for auto-turning
    public static double TURN_MAX_SPEED = 1.0; // Max auto-turn speed
    public static double TURN_SPEED_CUTOFF = 0.03; // How slow we should turn before stopping
    public static double TURN_CORRECT_FACTOR = 1; // Our P constant for turning

    // How much the robot should turn while strafing
    public static double SLEW_TURN_FACTOR = 0.2;

    // How long the triggers need to be held down before the macros
    // will kick in. They will have to hold down the triggers for this many MS
    // with at least the power stated below
    public static int MS_USE_DOWN_MACROS = 500;
    public static int MS_USE_UP_MACROS = 500;
    public static double POWER_USE_UP_DOWN_MACROS = 0.3;

    // How far up the winch should go
    public static int WINCH_MAX_POS = 6700;


    public ControlMapping controller;
    public boolean fieldCentric;

    private boolean wasTurningTo255;
    private double headingOffset;

    SparkyTheRobot robot;
    ElapsedTime loopTime;
    ElapsedTime timeMovingArmDown;
    ElapsedTime timeMovingArmUp;
    HoldingPIDMotor winch;
    Extender extender;

    Arm arm;

    @Override
    public void runOpMode() {

        robot = new SparkyTheRobot(this);
        robot.calibrate(true);
        headingOffset = 0;

        loopTime = new ElapsedTime();
        timeMovingArmDown = new ElapsedTime();
        timeMovingArmUp = new ElapsedTime();
        wasTurningTo255 = false;

        winch = new HoldingPIDMotor(robot.winch, 1);

        arm = new Arm(robot.leftFlipper, robot.rightFlipper, robot.linearSlide, robot.armIMU);
        extender = new Extender(robot.slideSwitch, robot.linearSlide);

        // Enable PID control on these motors
        robot.leftFlipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFlipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Keep standard front direction
        for (int i = 0; i < 4; i++) {
            if (i % 2 == 1) {robot.motorArr[i].setDirection(DcMotor.Direction.FORWARD);}
            else {robot.motorArr[i].setDirection(DcMotor.Direction.REVERSE);}
        }

        // Display setup readouts
        telemetry.log().clear();
        telemetry.log().add("Running RR2 TeleOp");
        telemetry.log().add("Control mapping: [[" + controller.getClass().getSimpleName() + "]]");
        telemetry.log().add("Relativity     : [[" + (fieldCentric ? "Field" : "Robot") + " centric]]");
        telemetry.update();


        // Intake flipper servos are disabled by default
        waitForStart();
        robot.markerDeployer.setPosition(AutoUtils.MARKER_DEPLOYER_RETRACTED);
        robot.parkingMarker.setPosition(AutoUtils.PARKING_MARKER_RETRACTED);
        loopTime.reset();

        while (opModeIsActive()) {
            controller.update();

            // For macro move up and down, perform shortcuts
            if (controller.collectWithArm()) {
                arm.collect();
                robot.intake.collect();
                controller.setIntakeDir(-1);
                extender.goToCollect();
            } else if (controller.depositWithArm()) {
                arm.deposit();
                controller.setIntakeDir(1);
                extender.goToMax();
            } else {
                double desiredArmSpeed = controller.armSpeed();
                arm.setPower(desiredArmSpeed);
                if (desiredArmSpeed > 0) {
                    robot.intake.collect();
                }
                if (desiredArmSpeed <= POWER_USE_UP_DOWN_MACROS) {
                    timeMovingArmDown.reset();
                }
                if (desiredArmSpeed >= -POWER_USE_UP_DOWN_MACROS) {
                    timeMovingArmUp.reset();
                }
            }

            if (timeMovingArmUp.milliseconds() > MS_USE_UP_MACROS) {
                extender.goToMax();
                controller.setIntakeDir(1);
            }
            if (timeMovingArmDown.milliseconds() > MS_USE_DOWN_MACROS) {
                extender.goToCollect();
            }

            if (controller.shakeCamera()) {
                robot.cameraPositioner.flipUp();
            } else {
                robot.cameraPositioner.flipDown();
            }

            // Check to make sure
            int winchPower = controller.getHangDir();
            if (!robot.hangSwitch.getState() && !controller.override()) {
                // If the switch is pressed (if it's not open)
                winchPower = Math.max(winchPower, 0);
            }

            winch.setPower(winchPower);
            if (controller.flipOut()) {robot.intake.collect();}
            else if (controller.flipBack()) {
                robot.intake.deposit();
                controller.setIntakeDir(-1);
            } else if (controller.armSpeed() < 0) {robot.intake.collect();}

            robot.intake.setIntakeSpeed(controller.getSpinSpeed());

            WheelDriveVector speeds = new WheelDriveVector(controller.driveStickY(), controller.driveStickX(), controller.turnSpeed());
            speeds.scale(controller.translateSpeedScale(), controller.turnSpeedScale());

            // Control linear slide extend retract and drive robot if necessary
            double slidePower = controller.getExtendSpeed();
            extender.setPower(slidePower);

            if (
                    ((extender.minExtend() && slidePower < 0) ||
                    (extender.maxExtend() && slidePower > 0)) &&
                    arm.isCollecting()) { // Don't move robot if we're not collecting
                speeds.forwardSpeed += slidePower * EXTEND_MAXED_DRIVE_POWER;
            }

            // Slew drive mapped to GP2 left/right
            speeds.translateSpeed += controller.getSlewSpeed();
            speeds.turnSpeed += controller.getSlewSpeed() * SLEW_TURN_FACTOR;
            speeds.turnSpeed += controller.getGP2TurnSpeed();

            if (controller.resetHeading()) {
                headingOffset = robot.getHeading() + HEADING_RESET_POSITION;
            }
            // Control heading locking
            if (controller.lockTo45() || controller.lockTo225()) {
                // Pressing y overrides lock to 45
                double targetAngle = Math.PI * 1.75;
                if (controller.lockTo225()) {
                    targetAngle = Math.PI * 0.75;
                }

                double difference = robot.getSignedAngleDifference(targetAngle, robot.normAngle(robot.getHeading() - headingOffset));
                double turnSpeed = Math.max(-TURN_MAX_SPEED, Math.min(TURN_MAX_SPEED,
                        difference * TURN_CORRECT_FACTOR));
                turnSpeed = Math.copySign(Math.max(TURN_SPEED_CUTOFF, Math.abs(turnSpeed)), turnSpeed);
                speeds.turnSpeed = -turnSpeed;
            }

            if (controller.lockTo225() && !wasTurningTo255) {
                wasTurningTo255 = true;
                winch.setTargetPos(WINCH_MAX_POS);
            } else if (wasTurningTo255 && !controller.lockTo225()) {
                wasTurningTo255 = false;
            }

            robot.setMotorSpeeds(speeds.getDrivePowers());

            // Telemetry
            //feedback.updateTelemetry(telemetry);
            int pos = (robot.leftFlipper.getCurrentPosition() + robot.rightFlipper.getCurrentPosition()) / 2;
            telemetry.addData("Arm position", pos);
            telemetry.addData("Drive stick y", controller.driveStickY());
            telemetry.addData("Drive stick actual y", gamepad1.left_stick_y);
            telemetry.addData("Extender position", robot.linearSlide.getCurrentPosition());
            telemetry.addData("Acc arm position", arm.getPositionRadians());
            telemetry.addData("Mag switch", robot.slideSwitch.getState());

            telemetry.addData("Winch pos", robot.winch.getCurrentPosition());
            telemetry.addData("Loop time", loopTime.milliseconds());
            loopTime.reset();
            telemetry.update();
        }
    }
}
