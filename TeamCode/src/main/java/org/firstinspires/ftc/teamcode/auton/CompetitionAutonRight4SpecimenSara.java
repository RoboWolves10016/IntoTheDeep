package org.firstinspires.ftc.teamcode.auton;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.paths.AutonPathsRight4HSara;
import org.firstinspires.ftc.teamcode.hardware.RobotHardwareC;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
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
@Autonomous (name = "Comp Auton Right 4 Specimen Sara" , group = "Autonomous")
public class CompetitionAutonRight4SpecimenSara extends OpMode {
    AutonPathsRight4HSara autonp = new AutonPathsRight4HSara(this);
    RobotHardwareC robot = new RobotHardwareC(this);

    static final int COMPLETE = -1, BEGIN_SUBSTATE = 10;
    static final Double CLIP_TIME = 1.0, BAR_DIST = 35.0, CLAW_OPEN_TIME = 0.4;
    static final int HOOK_HIEGHT = 1080;
    static final double JIGGLE_DEGREES = 3, JIGGLE_TIME = 0.01, READY_PUSH = 0.5;

    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private final boolean forward = true;

    public Follower follower;

    // TODO: adjust this for each auto
    private final Pose startPose = new Pose(7.25, 89.25, Math.toRadians(180));

    private int pathState, subPathState;

    private final ElapsedTime pausetime = new ElapsedTime();
    boolean backed = false;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        autonp.init(follower);
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
                setPathState(2);
                break;
            case 2: //
                follower.update();
                if (robot.liftBar() && !follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10: // move up a little to put sample against the bar
                follower.followPath(autonp.move1b);
                setPathState(12);
                setSubPathState(BEGIN_SUBSTATE);
                break;
            case 12: //
                hangSpecimen();
                if (subPathState == COMPLETE) {setPathState(18);}
                break;
//////////////////////////// 1st Sample Hooked
// push 2 samples into the zone
            case 18: // move to wall for 1st sample
                follower.followPath(autonp.move2a);
                setPathState(20);
                break;
            case 20:
                follower.update();
                robot.slideOut();
                robot.pushPos();
                if (!follower.isBusy()) { //|| follower.getPose().getX() > 66
                    follower.setMaxPower(1);
                    setPathState(22);
                }
                break;
            case 22:
                robot.slideOut();
                robot.liftWall();
                robot.pushPos();
                if (pausetime.seconds() > READY_PUSH) {
                    follower.followPath(autonp.move2b);
                    setPathState(23);
                }
                break;
            case 23:
                follower.update();
                if (!follower.isBusy()) {
                    setPathState(24);
                }
                break;
            // move to 2nd sample
            case 24:
                robot.liftPos();
                follower.followPath(autonp.move2c);
                setPathState(25);
                break;
            case 25:
                follower.update();
                if (!follower.isBusy()) {
                    robot.pushPos();
                    setPathState(26);
                }
                break;
            case 26:
                robot.pushPos();
                if (pausetime.seconds() > READY_PUSH) {
                    follower.followPath(autonp.move2d);
                    setPathState(27);
                }
                break;
            case 27:
                follower.update();
                if (!follower.isBusy()) {
                    setPathState(28);
                }
                break;
            // move to 3rd sample
            case 28:
                robot.idlePos();
                follower.followPath(autonp.move2e);
                setPathState(29);
                break;
            case 29:
                follower.update();
                if (!follower.isBusy()) {
                    robot.pushPos();
                    setPathState(30);
                }
                break;
            case 30:
                robot.pushPos();
                if (pausetime.seconds() > READY_PUSH) {
                    follower.followPath(autonp.move2f);
                    setPathState(31);
                }
                break;
            case 31:
                follower.update();
                if (!follower.isBusy()) {
                    setPathState(32);
                }
                break;
            case 32:
                robot.slideIn();
                robot.idlePos();
                follower.followPath(autonp.move2g);
                setPathState(33);
                break;
            case 33:
                follower.update();
                if (!follower.isBusy()) {
                    setPathState(34);
                }
                break;
            case 34:
                follower.followPath(autonp.move2h);
                setPathState(35);
                break;
            case 35:
                follower.update();
                if (!follower.isBusy()) {
                    setPathState(38);
                }
                break;
// pickup 2nd specimen
            case 38:
                robot.clawClosed();
                setPathState(39);
                break;
            case 39:
                if (pausetime.seconds() > 0.2) {
                    robot.liftBar();
                    setPathState(46);
                }
                break;
            case 46:
                follower.followPath(autonp.move3a);
                setPathState(48);
                break;
            case 48: //
                follower.update();
                if (!follower.isBusy() ) {
                    setPathState(50);
                }
                break;
            case 50: // move against bar
                follower.followPath(autonp.move3b);
                setPathState(52);
                setSubPathState(BEGIN_SUBSTATE);
                break;
            case 52: //
                hangSpecimen();
                if (subPathState == COMPLETE) {setPathState(60);}
                break;
////////////////////// 2nd sample hooked
            case 60: // move back for 3rd sample
                follower.followPath(autonp.move4b);
                setPathState(71);
                break;
            case 71: // follow path to align with 3rd specimen
                robot.liftWall();
                if (follower.getCurrentPathNumber() == 1) robot.liftWall();
                follower.update();
                if (!follower.isBusy()) {
                    setPathState(80);
                }
                break;
            case 80: // grab 3rd specimen
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move4c);
                    setPathState(82);
                }
                break;
            case 82: //
                follower.update();
                if (!follower.isBusy() || pausetime.seconds() > 0.5) {
                    setPathState(84);
                }
                break;
            case 84:
                robot.clawClosed();
                setPathState(86);
                break;
            case 86:
                if (pausetime.seconds() > 0.2) {
                    robot.liftBar();
                    setPathState(92);
                }
                break;
            case 92:
                follower.followPath(autonp.move5b);
                setPathState(94);
                break;
            case 94: //
                follower.update();
                if (!follower.isBusy() ) {
                    setPathState(96);
                }
                break;
            case 96:
                if (pausetime.seconds() > 0 ) {
                    follower.followPath(autonp.move5c);
                    setPathState(98);
                }
                break;
            case 98: // follow path to move specimen to bar
                follower.update();
                setPathState(100);
                setSubPathState(BEGIN_SUBSTATE);
                break;
            case 100: //
                hangSpecimen();
                if (subPathState == COMPLETE) {setPathState(102);}
                break;
////////////////////// 3rd sample hooked
            case 102:
                follower.followPath(autonp.move6b);
                setPathState(104);
                break;
            case 104: //
                robot.liftWall();
                if (follower.getCurrentPathNumber() == 1) robot.liftWall();
                follower.update();
                if (!follower.isBusy()) {
                    setPathState(110);
                }
                break;
            case 110: // grab 3rd specimen
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move6c);
                    setPathState(112);
                }
                break;
            case 112: //
                follower.update();
                if (!follower.isBusy() || pausetime.seconds() > 0.5) {
                    setPathState(114);
                }
                break;
            case 114:
                robot.clawClosed();
                setPathState(116);
                break;
            case 116:
                if (pausetime.seconds() > 0.2) {
                    robot.liftBar();
                    setPathState(122);
                }
                break;
            case 122:
                follower.followPath(autonp.move7b);
                setPathState(124);
                break;
            case 124: //
                follower.update();
                if (!follower.isBusy() ) {
                    setPathState(126);
                }
                break;
            case 126:
                if (pausetime.seconds() > 0 ) {
                    follower.followPath(autonp.move7c);
                    setPathState(128);
                }
                break;
            case 128: // follow path to move specimen to bar
                follower.update();
                setPathState(130);
                setSubPathState(BEGIN_SUBSTATE);
                break;
            case 130: //
                hangSpecimen();
                if (subPathState == COMPLETE) {setPathState(140);}
                break;
////////////////////// 3rd sample hooked
            case 140: // park
                robot.liftDown();
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move6b);
                    setPathState(150);
                }
                break;
            case 150: // follow path to park
                follower.update();
                if (!follower.isBusy()) {
                    setPathState(200);
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
    void hangSpecimen() {
        switch(subPathState) {
            case BEGIN_SUBSTATE:
                follower.update();
                if (!follower.isBusy() || follower.getPose().getX() > BAR_DIST) {
                    follower.holdPoint(new BezierPoint(follower.getCurrentPath().getLastControlPoint()), Math.toRadians(180));
                    robot.liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.liftL.setPower(-0.99);
                    robot.liftR.setPower(-0.99);
                    setSubPathState(14);
                }
                break;
            case 14:
                if (robot.liftL.getCurrentPosition() < HOOK_HIEGHT) {
                    robot.liftHook();
                    telemetryA.addLine("lift hook complete");
                    telemetryA.update();
                    setSubPathState(16);
                }
                break;
            case 16:
                robot.clawOpen();
                if (pausetime.seconds() > CLAW_OPEN_TIME) {
                    setSubPathState(COMPLETE);
                }
                break;
        }
    }
    public void jiggle() {
        double curX, curY;
        curX = follower.getPose().getX();
        curY = follower.getPose().getY();

        switch(subPathState) {
            case BEGIN_SUBSTATE:
                follower.holdPoint(new BezierPoint(new Point(curX, curY, Point.CARTESIAN)), Math.toRadians(JIGGLE_DEGREES));
                if (pausetime.seconds() > JIGGLE_TIME) {
                    setSubPathState(20);
                }
                break;
            case 20:
                follower.holdPoint(new BezierPoint(new Point(curX, curY, Point.CARTESIAN)), Math.toRadians(-JIGGLE_DEGREES));
                if (pausetime.seconds() > JIGGLE_TIME) {
                    setSubPathState(COMPLETE);
                }
                break;
        }

    }
    public void setPathState(int state) {
        pathState = state;
        pausetime.reset();
    }
    public void setSubPathState(int state) {
        subPathState = state;
        pausetime.reset();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}
