package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.paths.AutonPathsRight3H;
import org.firstinspires.ftc.teamcode.hardware.RobotHardwareC;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

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
@Autonomous (name = "Competition Auton Right Hang 3-2" , group = "Autonomous")
public class CompetitionAutonRight3Hang2 extends OpMode {
    AutonPathsRight3H autonp = new AutonPathsRight3H(this);
    RobotHardwareC robot = new RobotHardwareC(this);

    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private final boolean forward = true;

    public Follower follower;

    // TODO: adjust this for each auto
    private final Pose startPose = new Pose(7.25, 89.25, Math.toRadians(180));

    private int pathState;

    private final ElapsedTime pausetime = new ElapsedTime();
    public static PathChain move2a, move2b;
    boolean backed = false;


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        move2a = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(36,96, Point.CARTESIAN),
                        new Point(28,93, Point.CARTESIAN),
                        new Point(12,85, Point.CARTESIAN),
                        new Point(26,76, Point.CARTESIAN)))
                .addPath(new BezierCurve(
                        new Point(26, 76, Point.CARTESIAN),
                        new Point(40, 52, Point.CARTESIAN),
                        new Point(60, 77, Point.CARTESIAN),
                        new Point(62,65, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0)
                .build();
        move2b = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(36,96, Point.CARTESIAN),
                        new Point(24,96, Point.CARTESIAN),
                        new Point(20,95, Point.CARTESIAN),
                        new Point(20,80, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .addPath(new BezierLine(
                        new Point(20, 80, Point.CARTESIAN),
                        new Point(11,59, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0.5)
                .build();

        autonp.init();
        robot.init();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("I am " +
                this);
        telemetryA.addLine("This is our first trial moves for Into The Deep competition.");
        telemetryA.update();
        pathState = 1;
        follower.setStartingPose(startPose);
        follower.followPath(autonp.move1);
        robot.clawClosed();
        robot.undump();
        follower.setMaxPower(1);
        telemetryA.addLine("Claw closed");
        telemetryA.update();
        pausetime.reset();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {

        switch (pathState) {
            case 1:
                robot.idlePos();
                robot.slideIn();
                pausetime.reset();
                pathState = 2;
                break;
            case 2: //
                if (pausetime.seconds() > 0) {
                    follower.update();

                    if (robot.liftBar() && !follower.isBusy()) {
                        pausetime.reset();
                        pathState = 10;
                    }
                }
                break;
            case 10: // move up a little to put sample against the bar
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move1b);
                    pausetime.reset();
                    pathState = 12;
                }
                break;
            case 12: //
                follower.update();
                if (!follower.isBusy() || pausetime.seconds() > 0.4) {
                    telemetryA.addLine("Going to lifthook");
                    telemetryA.update();
                    pathState = 14;
                }

                break;
            case 14:
                if (robot.liftHook()) {
                    telemetryA.addLine("lift hook complete");
                    telemetryA.update();
                    pathState = 16;
                    pausetime.reset();
                }
                break;
            case 16:
                robot.clawOpen();
                if (pausetime.seconds() > 0.05) {
                    pausetime.reset();
                    pathState = 18;
                }
                break;
//////////////////////////// 1st Sample Hooked
            case 18: // move to wall for 1st sample
                if (pausetime.seconds() > 0.5) {
                    follower.followPath(move2a);
                    pausetime.reset();
                    pathState = 20;
                }
            break;
            case 20:
                follower.update();
                robot.liftWall();
                if (!follower.isBusy()) { //|| follower.getPose().getX() > 66
                    pathState = 21;
                    pausetime.reset();
                }
                break;
            case 21: // move 1st sample to the wall
                follower.followPath(autonp.move3);
                pathState = 22;
                break;
            case 22:
                follower.update();
                if (!follower.isBusy()) { // || follower.getPose().getX() < 12
                    pathState = 25;
                }
                break;
           /* case 23: // move 2nd sample to the wall
                follower.followPath(autonp.move4);
                pathState = 24;
                break;
            case 24:
                follower.update();
                if (!follower.isBusy() ) { // || follower.getPose().getX() < 12)
                    pathState = 25;
                }
                break;
          */  case 25: // back up so specimen can be place
                follower.followPath(autonp.move4b);
                robot.liftWall();
                pathState = 26;
                pausetime.reset();
                break;
            case 26:
                follower.update();
                if (!follower.isBusy() && pausetime.seconds() > 1) {
                    pathState = 27;
                    pausetime.reset();
                }
                break;
            case 27:
                follower.followPath(autonp.move5);
                robot.liftWall();
                pathState = 28;
                pausetime.reset();
                break;
            case 28:
                follower.update();
                if (!follower.isBusy() && pausetime.seconds() > 1) {
                    pathState = 30;
                    pausetime.reset();
                }
                break;
            case 30:
                follower.update();
                robot.liftWall();
                if (!follower.isBusy()) { //|| follower.getPose().getX() > 66
                    pathState = 32;
                    pausetime.reset();
                }
                break;
            case 32: // move to wall to pick up 2nd specimen
                if (pausetime.seconds()>1) {
                    follower.followPath(autonp.move5bh3); // ****** 1st move to wall
                    pausetime.reset();
                    pathState = 33;
                }
                break;
            case 33: // grab 2nd specimen
                follower.update();
                if (!follower.isBusy() || pausetime.seconds() > 0.7) {
                    pathState = 34;
                }
                break;
            case 34:
                robot.clawClosed();
                pausetime.reset();
                pathState = 36;
                break;
            case 36:
                if (pausetime.seconds() > 0.2) {
                    robot.liftBar();
                    pausetime.reset();
                    pathState = 46;
                }
                break;
         /*   case 40: // move back
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move6h31);
                    pathState = 41;
                }
                break;
            case 41: //
                follower.update();
                if (!follower.isBusy()) { // || follower.getPose().getX() > 13
                    pathState = 42;
                }
                break;
            case 42: // move back
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move6h32);
                    pathState = 44;
                }
                break;

            case 44: // move back
                follower.update();
                if (!follower.isBusy()) { // || follower.getPose().getX() > 13
                    pathState = 46;
                }
                break;

          */
            case 46:
                    follower.followPath(autonp.move6bh3);
                    pathState = 48;
                break;
            case 48: //
                follower.update();
                if (!follower.isBusy() ) {
                    pathState = 50;
                }
                break;
            case 50: // move against bar
                if (pausetime.seconds() > 0 ) {
                    follower.followPath(autonp.move6ch3);
                    pathState = 51;
                    pausetime.reset();
                }
                break;
            case 51: // follow path to move 2nd specimen to bar
                follower.update();
                if (!follower.isBusy() || pausetime.seconds() > 0.8) {
                    pathState = 52;
                }
                break;
            case 52:
                if (robot.liftHook()) {
                    telemetryA.addLine("lift hook complete");
                    telemetryA.update();
                    pathState = 54;
                    pausetime.reset();
                }
                break;
            case 54:
                robot.clawOpen();
                if (pausetime.seconds() > 0.3) {
                    pausetime.reset();
                    pathState = 60;
                }
                break;
////////////////////// 2nd sample hooked
            case 60: // move back for 3rd sample
                    follower.followPath(autonp.move7h3);
                    pathState = 61;
                break;
            case 61: //
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 70;
                }
                break;
            case 70: //
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move7bh3);
                    pathState = 71;
                    pausetime.reset();
                }
                break;
            case 71: // follow path to align with 3rd specimen
                robot.liftWall();
                follower.update();
                if (!follower.isBusy()) {
                    pausetime.reset();
                    pathState = 80;
                }
                break;
            case 80: // grab 3rd specimen
                if (pausetime.seconds() > 0.7) {
                    follower.followPath(autonp.move7ch3);
                    pathState = 82;
                    pausetime.reset();
                }
                break;
            case 82: //
                follower.update();
                if (!follower.isBusy() || pausetime.seconds() > 1.2) {
                    pathState = 84;
                }
                break;
            case 84:
                robot.clawClosed();
                pausetime.reset();
                pathState = 86;
                break;
            case 86:
                if (pausetime.seconds() > 0.2) {
                    robot.liftBar();
                    pausetime.reset();
                    pathState = 88;
                }
                break;
            case 88: // move back
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move8h3);
                    pathState = 90;
                }
                break;

            case 90: // move to bar with 3rd specimen
                follower.update();
                if (!follower.isBusy()) { // || follower.getPose().getX() > 13
                    pathState = 92;
                }
                break;
            case 92:
                follower.followPath(autonp.move8bh3);
                pathState = 94;
                break;
            case 94: //
                follower.update();
                if (!follower.isBusy() ) {
                    pathState = 96;
                }
                break;
            case 96:
                if (pausetime.seconds() > 0 ) {
                    follower.followPath(autonp.move8ch3);
                    pathState = 98;
                    pausetime.reset();
                }
                break;
            case 98: // follow path to move specimen to bar
                follower.update();
                if (!follower.isBusy()  || pausetime.seconds() > 0.4) {
                    pathState = 100;
                }
                break;
            case 100:
                if (robot.liftHook()) {
                    telemetryA.addLine("lift hook complete");
                    telemetryA.update();
                    pathState = 102;
                    pausetime.reset();
                }
                break;
            case 102:
                robot.clawOpen();
                if (pausetime.seconds() > 0.3) {
                    pausetime.reset();
                    pathState = 104;
                }
                break;
            case 104:
                follower.followPath(autonp.move9h3);
                pathState = 106;
                break;
            case 106: // backup a little
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 108;
                }
                break;
            case 108: // park
                robot.liftDown();
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move9bh3);
                    pathState = 110;
                }
                break;
            case 110: // follow path to park
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 200;
                }
                break;
            case 200:
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + pathState);
        }
        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}
