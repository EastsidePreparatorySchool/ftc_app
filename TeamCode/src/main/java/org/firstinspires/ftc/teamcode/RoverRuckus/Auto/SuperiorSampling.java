package org.firstinspires.ftc.teamcode.RoverRuckus.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DriveSystems.Mecanum.RoadRunner.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.Mechanisms.SparkyTheRobot;
import org.firstinspires.ftc.teamcode.RoverRuckus.Deployers.Auto.EndGoal;
import org.firstinspires.ftc.teamcode.RoverRuckus.Deployers.Auto.StartingPosition;
import org.firstinspires.ftc.teamcode.Utilities.RoadRunner.AssetsTrajectoryLoader;

import java.io.IOException;
@Config
public abstract class SuperiorSampling extends AutoUtils {

    public EndGoal goal;
    public static int position = -1;

    public void switchAppendagePositions() {
        robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.winch.setPower(1);
        robot.winch.setTargetPosition(-1500);

        robot.intake.collect();

        robot.sleep(2500);

        robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.winch.setPower(0);
        refoldMechanisms();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Set up road runner
        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);
        robot = new SparkyTheRobot(this);
        robot.calibrate(true);
        robot.markerDeployer.setPosition(MARKER_DEPLOYER_RETRACTED);
        robot.parkingMarker.setPosition(PARKING_MARKER_RETRACTED);
        initVuforia();
        setWinchHoldPosition();

        // Display telemetry feedback
        telemetry.log().add("Running sophisticated sampling op-mode");
        telemetry.log().add("Starting from: [[" + startingPosition.name() + "]]");
        telemetry.log().add("Final actions: [[" + goal.name() + "]]");
        telemetry.update();

        GoldPosition goldLoc = waitAndWatchMinerals();
        if (isStopRequested()) return;

        if (position == 0) {
            goldLoc = GoldPosition.LEFT;
        } else if (position == 1) {
            goldLoc = GoldPosition.CENTER;
        } else if (position == 2) {
            goldLoc = GoldPosition.RIGHT;
        }

        // Use appropriate method for dehooking
        double[] possibleHeadings = {Math.PI * 0.5, Math.PI * 0.75, Math.PI};
        if (startingPosition == StartingPosition.DEPOT) {
            unhookFromLander(drive, robot, possibleHeadings[goldLoc.index]);
        } else {
            unhookFromLander(drive, robot, Math.PI * 1.75);
        }
        telemetry.log().add("Heading before driving: " + robot.getGyroHeading());

        robot.cameraPositioner.flipDown();
        try {
            if (startingPosition == StartingPosition.DEPOT) {
                followPath(drive, AssetsTrajectoryLoader.load("Depo" + goldLoc.fileName + "Sel"));
                turnToPos(0);
                followPath(drive, Paths.STRAFE_LEFT);
                robot.markerDeployer.setPosition(MARKER_DEPLOYER_DEPLOY);
                followPath(drive, Paths.DEPOT_TO_OTHER_CRATER);
                // Deploy whacker after we finish driving
                // to avoid accidentally whacking our minerals
                robot.parkingMarker.setPosition(PARKING_MARKER_EXTENDED);
                switchAppendagePositions();
                robot.sleep(1000);
            } else {
                followPath(drive, AssetsTrajectoryLoader.load("Crater" + goldLoc.fileName + "Sel"));
                stopMoving();
                if (goal == EndGoal.BLUE_CRATER || goldLoc == GoldPosition.RIGHT) {
                    switchAppendagePositions();
                }
                turnToPos(Math.PI * 1.75);
                followPath(drive, AssetsTrajectoryLoader.load("CraterBackup"));
                turnToPos(Math.PI/4);
                followPath(drive, AssetsTrajectoryLoader.load("Crater" + goldLoc.fileName + "Dir"));

                if (goal == EndGoal.BLUE_DOUBLE_SAMPLE || goldLoc == GoldPosition.RIGHT) {
                    if (goldLoc == GoldPosition.LEFT) {
                        turnToPos(3 * Math.PI / 2);
                    } else if (goldLoc == GoldPosition.CENTER) {
                        turnToPos(5 * Math.PI / 4);
                        // Strafe diagonally into the corner to align ourselves
                        followPath(drive, Paths.CRATER_DOUBLE_CENTER_DIAGONAL_STRAFE);
                        turnToPos(5 * Math.PI / 4);
                    } else {
                        turnToPos(0);
                        followPath(drive, Paths.STRAFE_RIGHT);
                        followPath(drive, Paths.FORWARD);

                        robot.markerDeployer.setPosition(MARKER_DEPLOYER_DEPLOY);
                    }
                    followPath(drive, AssetsTrajectoryLoader.load("CraterDouble" + goldLoc.fileName));

                    if (goldLoc != GoldPosition.RIGHT) {
                        turnToPos(0);
                        followPath(drive, Paths.DOUBLE_RETURN_TO_DEPO);
                        robot.markerDeployer.setPosition(MARKER_DEPLOYER_DEPLOY);
                        robot.parkingMarker.setPosition(PARKING_MARKER_EXTENDED);
                        robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.winch.setPower(1);
                        robot.winch.setTargetPosition(-1500);
                        robot.intake.collect();
                        followPath(drive, Paths.DEPOT_TO_SAME_CRATER_SHORT);
                        robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.winch.setPower(0);
                        refoldMechanisms();
                        robot.sleep(500);
                    } else {
                        robot.parkingMarker.setPosition(PARKING_MARKER_EXTENDED);
                    }
                } else {
                    turnToPos(0);
                    robot.markerDeployer.setPosition(MARKER_DEPLOYER_DEPLOY);
                    followPath(drive, Paths.STRAFE_RIGHT);
                    // Deploy marker here to ensure that if we time out, we
                    // can slide into crater
                    robot.parkingMarker.setPosition(PARKING_MARKER_EXTENDED);
                    followPath(drive, Paths.DEPOT_TO_SAME_CRATER);
                }
            }
        } catch (IOException e) {
            // TODO retry load
            e.printStackTrace();
        }

        // Now we're parked by crater
    }
}
