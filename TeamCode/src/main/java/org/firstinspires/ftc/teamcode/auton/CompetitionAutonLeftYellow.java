package org.firstinspires.ftc.teamcode.auton;

import android.telephony.SmsMessage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.paths.AutonPathsLeft;
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
@Autonomous (name = "Competition Auton Left Yellow" , group = "Autonomous")
public class CompetitionAutonLeftYellow extends OpMode {
    AutonPathsLeft autonp = new AutonPathsLeft(this);
    RobotHardwareC robot = new RobotHardwareC(this);

    static final int COMPLETE = -1, BEGIN_SUBSTATE = 10;
    static final Double UNDUMP_TIME = 1.0, TRANSFER_TIME = 0.5, DROP_TIME = 0.5, CLEAR_BASKET_TIME = 1.5;

    private Telemetry telemetryA;
    public static double DISTANCE = 40;
    private final boolean forward = true;

    private Follower follower;

    // TODO: adjust this for each auto
 //   private final Pose startPose = new Pose(7.25 + 0.72, 89.25 + 0.51, Math.toRadians(90));
    private final Pose startPose = new Pose(7.25, 89.25, Math.toRadians(180));
    private int pathState, subPathState;

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
        pathState = 101;
        follower.setStartingPose(startPose);
        follower.followPath(autonp.move1y);
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
            case 101: // starts following the first path to the basket
                robot.slideIn();
                robot.clawOpen();
                robot.idlePos();
                robot.liftBasket();
                follower.update();
                if (!follower.isBusy()) {
                    setPathState(10);;
                    setSubPathState(BEGIN_SUBSTATE);
                }
                break;
            case 10:  // In front of baskets - begin deposit sample process
                dumpSample();
                if (subPathState == COMPLETE) {setPathState(18);}
                break;
//////////////////////// after 1st dump
            case 18:  // move forward to eat 1st sample
                if (pausetime.seconds() > 0.0) {
                    robot.intakePos();
                    robot.spinIn();
                    follower.followPath(autonp.move2ay);
                    setPathState(20);
                }
                break;
            case 20:
                follower.update();
                robot.intakePos();
                robot.spinIn();
                if (pausetime.seconds() > CLEAR_BASKET_TIME) {robot.liftDown();}
                if (!follower.isBusy()) {
                    robot.liftDown();
                    setPathState(21);
                }
                break;
            case 21: // drive angled in towards the 1st sample
                follower.followPath(autonp.move2b);
                setPathState(22);;
                break;
            case 22:
                follower.update();
                if (!follower.isBusy()) { //|| follower.getPose().getY() > 103
                    setPathState(30);
                }
                break;
            case 30: // move to the basket
                //robot.spinOff();
                robot.transferPos();
                follower.followPath(autonp.move3b);
                setPathState(33);
                break;
            case 33: // to basket
                follower.update();
                if (pausetime.seconds() > TRANSFER_TIME) {
                    robot.spinOut();
                }
                if (pausetime.seconds() > TRANSFER_TIME + DROP_TIME) {
                    robot.spinOff();
                    robot.idlePos();
                    robot.liftBasket();
                }
                if (!follower.isBusy()) {
                    setSubPathState(BEGIN_SUBSTATE);
                    setPathState(34);
                }
                break;
            case 34:
                robot.spinOff();
                robot.idlePos();
                dumpSample();
                if (subPathState == COMPLETE) {
                    setSubPathState(BEGIN_SUBSTATE);
                    setPathState(40);}
                break;
//////////////////////// after 2nd dump
            case 40: // to 2nd sample
                robot.intakePos();
                robot.spinIn();
                follower.followPath(autonp.move4);
                setPathState(44);
                break;

            case 44:
                follower.update();
                if (pausetime.seconds() > CLEAR_BASKET_TIME) {robot.liftDown();}
                if (!follower.isBusy()) {
                    robot.liftDown();
                    setPathState(46);
                }
                break;
            case 46:
                    follower.followPath(autonp.move4a);
                    setPathState(48);
                break;
            case 48: //
                follower.update();
                if (robot.liftDown() && !follower.isBusy()) {
                    setPathState(50);
                }
                break;
            case 50: // to basket
                robot.transferPos();
                pausetime.reset();
                follower.followPath(autonp.move5);
                setPathState(52);
                break;
            case 52: //
                follower.update();
                if (pausetime.seconds() > TRANSFER_TIME) {
                    robot.spinOut();
                }
                if (pausetime.seconds() > TRANSFER_TIME + DROP_TIME) {
                    robot.spinOff();
                    robot.idlePos();
                    robot.liftBasket();
                }
                if (!follower.isBusy()) {
                    setSubPathState(BEGIN_SUBSTATE);
                    setPathState(54);
                }
                break;
            case 54:
                robot.spinOff();
                robot.idlePos();
                dumpSample();
                if (subPathState == COMPLETE) {
                    setSubPathState(BEGIN_SUBSTATE);
                    setPathState(60);
                }
                break;
//////////////////////// after 3rd dump
            case 60: // to 3rd spike mark
                // robot.slideOut();
                robot.intakePos();
                robot.spinIn();
                follower.followPath(autonp.move6);
                setPathState(64);
                break;

            case 64:
                follower.update();
                if (pausetime.seconds() > CLEAR_BASKET_TIME) {robot.liftDown();}
                if (!follower.isBusy()) {
                    robot.liftDown();
                    setPathState(66);
                }
                break;
            case 66:
                follower.followPath(autonp.move6a);
                setPathState(68);
                break;
            case 68: // starts following the first path to score on the spike mark
                follower.update();
                if (robot.liftDown() && !follower.isBusy()) {
                    setPathState(70);
                }
                break;
            case 70: // to basket
              //  robot.spinOff();
                robot.transferPos();
                pausetime.reset();
                follower.followPath(autonp.move7);
                setPathState(72);
                pausetime.reset();
                break;
            case 72: //
                follower.update();
                if (pausetime.seconds() > TRANSFER_TIME) {
                    robot.spinOut();
                }
                if (pausetime.seconds() > TRANSFER_TIME + DROP_TIME) {
                    robot.spinOff();
                    robot.idlePos();
                    robot.liftBasket();
                }
                if (!follower.isBusy()) {
                    setSubPathState(BEGIN_SUBSTATE);
                    setPathState(74);
                }
                break;
            case 74:
                robot.spinOff();
                robot.idlePos();
                dumpSample();
                if (subPathState == COMPLETE) {
                    setSubPathState(BEGIN_SUBSTATE);
                    setPathState(80);
                }
                break;
//////////////////////// after 4th dump
            case 80:
                follower.followPath(autonp.move8);
                setPathState(82);  // was 68 ????
                break;
            case 82: // starts following the first path to score on the spike mark
                follower.update();
                if (pausetime.seconds() > CLEAR_BASKET_TIME) {robot.liftDown();}
                if (!follower.isBusy()) {
                    robot.liftDown();
                    setPathState(100);
                }
                break;
            case 100:
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + pathState);
        }
        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
    void dumpSample() {
        switch(subPathState) {
            case BEGIN_SUBSTATE:
                if (robot.liftBasket()) {
                    telemetryA.addLine("lift basket complete");
                    telemetryA.update();
                    setSubPathState(12);
                }
                break;
            case 12:
                robot.dump();
                pausetime.reset();
                setSubPathState(14);
                break;
            case 14:
                if (pausetime.seconds() > UNDUMP_TIME) {
                    robot.undump();
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
}
