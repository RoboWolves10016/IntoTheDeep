package org.firstinspires.ftc.teamcode.auton;

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
@Autonomous (name = "Competition Auton Left Score 4" , group = "Autonomous")
public class CompetitionAutonLeftScore4 extends OpMode {
    AutonPathsLeft autonp = new AutonPathsLeft(this);
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
        follower.followPath(autonp.move1);
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
                robot.idlePos();
                robot.slideIn();
                follower.update();
                if (robot.liftBar() && !follower.isBusy()) {
                    pausetime.reset();
                    pathState = 10;
                }
                break;
            case 10:
                if (pausetime.seconds() > 0) {
                    follower.followPath(autonp.move1b);
                    pathState = 12;
                }
                break;
            case 12: // moves up against the bar
                follower.update();
                if (!follower.isBusy() || pausetime.seconds() > 0.4) {
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
                if (robot.clawOpen() && pausetime.seconds() > 0.2) {
                    pathState = 18;
                }
                break;
            case 18:  // move back and turn 90 deg
                if (pausetime.seconds() > 0.0) {
                    follower.followPath(autonp.move2a);
                    pathState = 20;
                }
                break;
            case 20:
                follower.update();
                robot.liftDown();
                robot.intakePos();
                robot.spinIn();
                if (!follower.isBusy()) {
                    pathState = 21;
                    pausetime.reset();
                }
                break;
            case 21: // drive angled in towards the 1st sample
                follower.followPath(autonp.move2b);
                pathState = 22;
                break;
            case 22:
                follower.update();
                if (!follower.isBusy()) { //|| follower.getPose().getY() > 103
                    pathState = 30;
                }
                break;
            case 30: // move to the basket
                //robot.spinOff();
                robot.transferPos();
                pausetime.reset();
                follower.followPath(autonp.move3b);
                pathState = 33;
                break;
            case 33: // to basket
                follower.update();
                if (pausetime.seconds()>0.5) {
                    robot.spinOut();
                }
                if (!follower.isBusy()) {
                    pathState = 34;
                }
                break;
            case 34:
                robot.spinOff();
                robot.idlePos();
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
                if (pausetime.seconds() > 1) {
                    robot.undump();
                    if (pausetime.seconds() > 0.02) {
                        pathState = 40;
                    }
                }
                break;
//////////////////////// after 1st dump
            case 40: // to 2nd sample
               // robot.slideOut();
                robot.intakePos();
                robot.spinIn();
              //  robot.liftDown();
                follower.followPath(autonp.move4);
                pausetime.reset();
                pathState = 44;
                break;

            case 44:
                follower.update();
                if (pausetime.seconds() > 2.5) {robot.liftDown();}
                if (!follower.isBusy()) {
                    robot.liftDown();
                    pathState = 46;
                }
                break;
            case 46:
                    follower.followPath(autonp.move4a);
                    pathState = 48;
                break;
            case 48: //
                follower.update();
                if (robot.liftDown() && !follower.isBusy()) {
                    pausetime.reset();
                    pathState = 50;
                }
                break;
            case 50: // to basket
             //   robot.spinOff();
                robot.transferPos();
                if (pausetime.seconds() > 0.4) {
                    robot.spinOut();
                    follower.followPath(autonp.move5);
                    pathState = 52;
                }
                break;
            case 52: //
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 54;
                }
                break;
            case 54:
                robot.spinOff();
                robot.idlePos();
                if (robot.liftBasket()) {
                    telemetryA.addLine("lift basket complete");
                    telemetryA.update();
                    pathState = 56;
                }
                break;
            case 56: // puke in basket
                robot.dump();
                pausetime.reset();
                pathState = 58;
                break;
            case 58:
                if (pausetime.seconds() > 1 ) {
                    robot.undump();
                    if (pausetime.seconds() > 0.02) {
                        pathState = 60;
                    }
                }
                break;
//////////////////////// after 2nd dump
            case 60: // to 3rd spike mark
                // robot.slideOut();
                robot.intakePos();
                robot.spinIn();
                follower.followPath(autonp.move6);
                pathState = 64;
                break;

            case 64:
                follower.update();
                if (pausetime.seconds() > 2.5) {robot.liftDown();}
                if (!follower.isBusy()) {
                    robot.liftDown();
                    pathState = 66;
                }
                break;
            case 66:
                follower.followPath(autonp.move6a);
                pausetime.reset();
                pathState = 68;
                break;
            case 68: //
                follower.update();
                if (!follower.isBusy() || pausetime.seconds() > 1) {
                    pausetime.reset();
                    pathState = 70;
                }
                break;
            case 70: // to basket
              //  robot.spinOff();
                robot.transferPos();
                if (pausetime.seconds() > 0.3) {
                    follower.followPath(autonp.move7);
                    pathState = 72;
                    pausetime.reset();
                }
                break;
            case 72: //
                follower.update();
                if (pausetime.seconds() > 1) {
                    robot.spinOut();
                }
                if (!follower.isBusy()) {
                    pathState = 74;
                }
                break;
            case 74:
                robot.spinOff();
                robot.idlePos();
                if (robot.liftBasket()) {
                    telemetryA.addLine("lift basket complete");
                    telemetryA.update();
                    pathState = 76;
                }
                break;
            case 76: // puke in basket
                robot.dump();
                pausetime.reset();
                pathState = 78;
                break;
            case 78:
                if (pausetime.seconds() > 1) {
                    robot.undump();
                    pathState = 80;
                }
                break;
//////////////////////// after 3rd dump
            case 80:
                follower.followPath(autonp.move8);
                pathState = 82;  // was 68 ????
                break;
            case 82: // starts following the first path to score on the spike mark
                follower.update();
                if (pausetime.seconds() > 2.5) {robot.liftDown();}
                if (!follower.isBusy()) {
                    pausetime.reset();
                    robot.liftDown();
                    pathState = 100;
                }
                break;
            case 90:
                follower.followPath(autonp.move8);
                pathState = 92;
                break;
            case 92: //
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 100;
                }
            case 100:
                break;
        }
        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
