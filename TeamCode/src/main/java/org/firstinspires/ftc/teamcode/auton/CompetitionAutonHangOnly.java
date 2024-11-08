package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.paths.AutonPaths1;
import org.firstinspires.ftc.teamcode.hardware.RobotHardwareC;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Competition Auton Left Hang Only" , group = "Autonomous Pathing Tuning")
public class CompetitionAutonHangOnly extends OpMode {
    AutonPaths1 autonp = new AutonPaths1(this);
    RobotHardwareC robot = new RobotHardwareC(this);

    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private final boolean forward = true;

    private Follower follower;

    // TODO: adjust this for each auto
    private final Pose startPose = new Pose(7.25, 89.25, Math.toRadians(180));

    private int pathState;

    private final ElapsedTime pausetime = new ElapsedTime();


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        autonp.init();
        robot.init();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("I am " +
                this);
        telemetryA.addLine("This is our first trial moves for Into The Deep competition.");
        telemetryA.update();
        pathState = 1;
        follower.setStartingPose(startPose);
        follower.followPath(AutonPaths1.move1);
        robot.clawClosed();
        robot.undump();
        follower.setMaxPower(1);
        telemetryA.addLine("Claw closed");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        switch (pathState) {
            case 1: // starts following the first path to score a sample on the bar wait for lift to be up before moving on
                follower.update();
                if (robot.liftBar() && !follower.isBusy()) {
                    pausetime.reset();
                    pathState = 10;
                }
                break;
            case 10:
                if (pausetime.seconds() > 0 ) {
                    follower.followPath(AutonPaths1.move1b);
                    pathState = 12;
                }
                break;
            case 12: // moves up against the bar
                follower.update();
                if (!follower.isBusy()) {
                    telemetryA.addLine("Going to lifthook");
                    telemetryA.update();
                    pathState = 14;
                }

                break;
            case 14: // lowers lift to clip in hook
                if (robot.liftHook()) {
                    telemetryA.addLine("lift hook complete");
                    telemetryA.update();
                    pathState = 16;
                    pausetime.reset();
                }
                break;
            case 16: // open the claw
                if (pausetime.seconds() > 0.05 && robot.clawOpen() ) {
                    pathState = 18;
                }
                break;
            case 18:  // move back and turn 90 deg
                if (pausetime.seconds() > 0.2 ) {
                    follower.followPath(AutonPaths1.move2a);
                    pathState = 20;
                }
                break;
            case 20:
                follower.update();
                robot.liftDown();
                if (!follower.isBusy()) {
                 //   robot.spinIn();
                    pathState = 21;
                    pausetime.reset();
                }
                break;
            case 21: // drive parallel to the samples
                follower.followPath(AutonPaths1.move2b);
                pathState = 22;
                break;
            case 22:
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 23;
                }
                break;
            case 23: // drive sideways to align with the samples
                follower.followPath(AutonPaths1.move2c);
                pathState = 24;
                break;
            case 24:
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 100;
                }
                break;
            case 25:  // move forward so spinner will land on the sample
                follower.followPath(AutonPaths1.move2d);
                pathState = 26;
                pausetime.reset();
                break;
            case 26:
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 28;
                    pausetime.reset();
                }
                break;
            case 28: // lower the arm and start the spinner
                robot.armDown();
                robot.spinIn();
                if (pausetime.seconds() > 1) {
                    pathState = 29;
                    pausetime.reset();
                }
                break;
            case 29:
                if (pausetime.seconds() > 1.5 ) {
                    robot.spinOff();
                    if (robot.armUp()) {
                        pathState = 32; // skip to move3b
                        pausetime.reset();
                    }
                }
                break;
            case 30: // move to the basket
                if (pausetime.seconds() > 1) {
                    robot.spinIn();
                    follower.followPath(AutonPaths1.move3);
                    pathState = 31;
                }
                break;
            case 31: // to basket
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 32;
                }
                break;
            case 32: // move to the basket
                follower.followPath(AutonPaths1.move3b);
                pathState = 33;
                break;
            case 33: // to basket
                robot.spinIn();
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 34;
                }
                break;
            case 34:
                robot.spinOff();
                if (robot.liftBasket()) {
                    telemetryA.addLine("lift basket complete");
                    telemetryA.update();
                    pathState = 36;
                }
                break;
            case 36:
                robot.dump();
                pausetime.reset();
                pathState = 38;
                break;
            case 38:
                if (pausetime.seconds() > 1 ) {
                    robot.undump();
                    if (pausetime.seconds() > 1) {
                        pathState = 40;
                    }
                }
                break;
            case 40:
                follower.followPath(AutonPaths1.move4);
                pathState = 44;
                break;

            case 44:
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 46;
                }
                break;
            case 46:
                    robot.liftDown();
                    follower.followPath(AutonPaths1.move4a);
                    pathState = 48;
                break;
            case 48: // starts following the first path to score on the spike mark
                follower.update();
                if (robot.liftDown() && !follower.isBusy()) {
                    pathState = 100;
                }
                break;
            case 50:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(AutonPaths1.move5);
                    pathState = 51;
                }
                break;
            case 51: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 60;
                }
                break;
            case 60:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(AutonPaths1.move6);
                    pathState = 61;
                }
                break;
            case 61: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 70;
                }
                break;
            case 70:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(AutonPaths1.move7);
                    pathState = 71;
                }
                break;
            case 71: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 80;
                }
                break;
            case 80:
                if (pausetime.seconds() > 0.5 ) {
                    follower.setStartingPose(new Pose(-10, -35, -0.85));
                    follower.followPath(AutonPaths1.move8);
                    pathState = 81;
                }
                break;
            case 81: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 100;
                }
                break;
            case 100:
                break;
        }
        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
