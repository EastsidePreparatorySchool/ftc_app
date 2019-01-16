package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest60Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

/*
 * Hardware class for a rotary arm (for linearly-actuated mechanisms, see Elevator).
 */
@Config
public class Arm {
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest60Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static final BNO055IMU.Parameters metricParameters = new BNO055IMU.Parameters();
    static {
        metricParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        metricParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        metricParameters.magPowerMode = BNO055IMU.MagPowerMode.SLEEP;
    }

    public static double COLLECT_DIST = 0.4;

    public static double GEAR_RATIO = 60.0/256.0; // output/input

    // the operating range of the arm is [0, MAX_ANGLE]
    public static double MAX_ANGLE = Math.PI; // rad
    public static double VERT_ANGLE = Math.PI/2;

    public static PIDCoefficients PID = new PIDCoefficients(0, 0, 0);

    public static double MAX_VEL = Math.PI * 2; // rad/s, we will never hit this
    public static double MAX_ACCEL = 5; // rad/s^2 // This is our limiting factor
    public static double MAX_JERK = 100; // rad/s^3

    public static double kV = 0.45;
    public static double kA = 0;
    public static double kStatic = 0;


    private DcMotorEx leftMotor, rightMotor;
    private ServoImplEx leftIntakeFlipper, rightIntakeFlipper;
    private PIDFController controller;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredAngle = 0;
    private int leftOffset, rightOffset;
    private BNO055IMU primaryIMU;



    private static double encoderTicksToRadians(int ticks) {
        return 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI / 60.0;
    }

    public static double getMaxRpm() {
        return MOTOR_CONFIG.getMaxRPM();
    }

    public Arm(DcMotorEx leftMotor, DcMotorEx rightMotor, BNO055IMU primaryIMU) {

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // if necessary, reverse the motor so CCW is positive
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeFlipper.setDirection(ServoImplEx.Direction.FORWARD);
        rightIntakeFlipper.setDirection(ServoImplEx.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftIntakeFlipper.setPosition(COLLECT_DIST);
        rightIntakeFlipper.setPosition(COLLECT_DIST);

        primaryIMU.initialize(metricParameters);

        // note: if the arm is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it here (assuming no velocity PID) like so:
        controller = new PIDFController(PID, kV, kA, kStatic,
                                               angle -> kA * 9.81 * Math.sin(angle - VERT_ANGLE));

        leftOffset = leftMotor.getCurrentPosition();
        rightOffset = rightMotor.getCurrentPosition();
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public void setAngle(double angle) {
        angle = Math.min(Math.max(0, angle), MAX_ANGLE);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(desiredAngle, 0, 0, 0);
        MotionState goal = new MotionState(angle, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );
        profileStartTime = clock.seconds();

        this.desiredAngle = angle;
    }

    public double getCurrentAngle() {
        return primaryIMU.getAngularOrientation().toAxesOrder(AxesOrder.ZYX).firstAngle;
    }

    public void update() {
        double power;
        double currentAngle = getCurrentAngle();
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(currentAngle, state.getV(), state.getA());
        } else {
            // just hold the position
            controller.setTargetPosition(desiredAngle);
            power = controller.update(currentAngle);
        }
        setPower(power);
    }

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

}
