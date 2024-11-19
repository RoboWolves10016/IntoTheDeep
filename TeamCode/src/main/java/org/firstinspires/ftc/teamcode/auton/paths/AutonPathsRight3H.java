package org.firstinspires.ftc.teamcode.auton.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndHeadingConstraint;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndTValueConstraint;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndTimeoutConstraint;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndTranslationalConstraint;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndVelocityConstraint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class AutonPathsRight3H {;
;
    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    private Follower follower;

    public static PathChain move2;
    public static Path movepl;
    public static Path movepr;
    public static Path move1, move1b, move1c;
    public static Path move23h, move2a, move2b, move2c, move2d, move2e, move2b2, move2t, move2d2;
    public static Path move3, move3b;
    public static Path move4, move4b;
    public static Path move5, move5b, move5h3, move5bh3;
    public static Path move6, move6b, move6c, move6h31, move6h32, move6bh3, move6ch3;
    public static Path move7, move7h3, move7b, move7bh3, move7ch3;
    public static Path move8h3, move8bh3, move8ch3;
    public static Path move9h3, move9bh3;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutonPathsRight3H(OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the moves.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init()    {
        // to spot to hang specimen
        move1 = new Path(new BezierLine(
                new Point(7.25,87.75, Point.CARTESIAN),
                new Point(31.25,96, Point.CARTESIAN)));
        move1.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimin
        move1.setPathEndTimeoutConstraint(0);

        move1b = new Path(new BezierLine(
                new Point(31.25,96, Point.CARTESIAN),
                new Point(36.,96, Point.CARTESIAN)));
        move1b.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin
        move1b.setPathEndTimeoutConstraint(0);

          // push 1st sample to the wall
        move3 = new Path(new BezierCurve(
                new Point(62,65, Point.CARTESIAN),
                new Point(63,50, Point.CARTESIAN),
                new Point(20,59, Point.CARTESIAN)));
        move3.setConstantHeadingInterpolation(Math.toRadians(0)); // to wall
        move3.setPathEndTimeoutConstraint(0);

        // push 2nd sample to the wall
        move4 = new Path(new BezierCurve(
                new Point(10,54, Point.CARTESIAN),
                new Point(76,70, Point.CARTESIAN),
                new Point(76,42, Point.CARTESIAN),
                new Point(10,46, Point.CARTESIAN)));
        move4.setConstantHeadingInterpolation(Math.toRadians(0));
        move4.setPathEndTimeoutConstraint(0);// back up and push 2nd sample to wall

        // move back so specimen can be placed
        move4b = new Path(new BezierLine(
                new Point(10.,59, Point.CARTESIAN),
                new Point(27,59, Point.CARTESIAN)));
        move4b.setConstantHeadingInterpolation(Math.toRadians(0));  // to 2nd neutral specimen

        // move forward to align with specimen
        move5 = new Path(new BezierLine(
                new Point(27,59, Point.CARTESIAN),
                new Point(11,59, Point.CARTESIAN)));
        move5.setConstantHeadingInterpolation(0);// to basket

        // move to specimen
        move5b = new Path(new BezierLine(
                new Point(11,59, Point.CARTESIAN),
                new Point(8,48, Point.CARTESIAN)));
        move5b.setConstantHeadingInterpolation(0);
        // move to specimen
        move5bh3 = new Path(new BezierLine(
                new Point(11,59, Point.CARTESIAN),
                new Point(8,59, Point.CARTESIAN)));
        move5bh3.setConstantHeadingInterpolation(0);

        // back up to get away from wall
        move6 = new Path(new BezierCurve(
                new Point(8,48, Point.CARTESIAN),
                new Point(20,48, Point.CARTESIAN)));
        move6.setConstantHeadingInterpolation(Math.toRadians(0));
        move6.setPathEndTimeoutConstraint(0); // Back away from wall

        // swing around to push 1st specimen into zone
        move6h31 = new Path(new BezierCurve(
                new Point(8,59, Point.CARTESIAN),
                new Point(37,76, Point.CARTESIAN),
                new Point(62,67, Point.CARTESIAN)));
        move6h31.setConstantHeadingInterpolation(Math.toRadians(0));
        move6h31.setPathEndTimeoutConstraint(0);// back up and push 2nd sample to wall

        // push 1st sample to the wall
        move6h32 = new Path(new BezierCurve(
                new Point(62,67, Point.CARTESIAN),
                new Point(63,51, Point.CARTESIAN),
                new Point(20,54, Point.CARTESIAN)));
        move6h32.setConstantHeadingInterpolation(Math.toRadians(0));
        move6h32.setPathEndTimeoutConstraint(0);// back up and push 2nd sample to wall

        // move to hang 2nd specimen
        move6b = new Path(new BezierCurve(
                new Point(20,48, Point.CARTESIAN),
                new Point(31.25,99, Point.CARTESIAN)));
        move6b.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimen
        move6b.setPathEndTimeoutConstraint(0);

        move6bh3 = new Path(new BezierCurve(
                new Point(20,54, Point.CARTESIAN),
                new Point(31.25,99, Point.CARTESIAN)));
        move6bh3.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimen
        move6bh3.setPathEndTimeoutConstraint(0);
        // move against the bar to hang 2nd specimen
        move6ch3 = new Path(new BezierLine(
                new Point(31.25,99, Point.CARTESIAN),
                new Point(36.,99, Point.CARTESIAN)));
        move6ch3.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin
        // back up away from bar
        move7 = new Path(new BezierLine(
                new Point(36,99, Point.CARTESIAN),
                new Point(33,99, Point.CARTESIAN)));
        move7.setConstantHeadingInterpolation(180);
        move7.setPathEndTimeoutConstraint(0); //

        move7h3 = new Path(new BezierLine(
                new Point(36,99, Point.CARTESIAN),
                new Point(33,99, Point.CARTESIAN)));
        move7h3.setConstantHeadingInterpolation(180);
        move7h3.setPathEndTimeoutConstraint(0); // back away from bar

        // to park
        move7b = new Path(new BezierLine(
                new Point(33,99, Point.CARTESIAN),
                new Point(11,48, Point.CARTESIAN)));
        move7b.setConstantHeadingInterpolation(0);  // to park
        // pick up 3rd sample
        move7bh3 = new Path(new BezierLine(
                new Point(33,99, Point.CARTESIAN),
                new Point(11,59, Point.CARTESIAN)));
        move7bh3.setConstantHeadingInterpolation(0);  //

        move7ch3 = new Path(new BezierLine(
                new Point(11,59, Point.CARTESIAN),
                new Point(8,59, Point.CARTESIAN)));
        move7ch3.setConstantHeadingInterpolation(0);  // to park

        // back up to get away from wall
        move8h3 = new Path(new BezierCurve(
                new Point(11,59, Point.CARTESIAN),
                new Point(20,59, Point.CARTESIAN)));
        move8h3.setConstantHeadingInterpolation(Math.toRadians(0));
        move8h3.setPathEndTimeoutConstraint(0); // Back away from wall

        // move to hang 2nd specimen
        move8bh3 = new Path(new BezierCurve(
                new Point(20,59, Point.CARTESIAN),
                new Point(31.25,101, Point.CARTESIAN)));
        move8bh3.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimen
        move8h3.setPathEndTimeoutConstraint(0);

        // move against the bar to hang 2nd specimen
        move8ch3 = new Path(new BezierLine(
                new Point(31.25,101, Point.CARTESIAN),
                new Point(36.,101, Point.CARTESIAN)));
        move8ch3.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin

        // back up away from bar
        move9h3 = new Path(new BezierLine(
                new Point(36,101, Point.CARTESIAN),
                new Point(33,101, Point.CARTESIAN)));
        move9h3.setConstantHeadingInterpolation(180);
        move9h3.setPathEndTimeoutConstraint(0); // back away from bar

        // to park
        move9bh3 = new Path(new BezierLine(
                new Point(33,101, Point.CARTESIAN),
                new Point(11,48, Point.CARTESIAN)));
        move9bh3.setConstantHeadingInterpolation(0);  // to park

    } // init
    public void endFast()    {
        pathEndVelocityConstraint = 0.2;
        pathEndTranslationalConstraint = 0.2;
        pathEndHeadingConstraint = 0.02;
        pathEndTValueConstraint = 0.795;
        pathEndTimeoutConstraint = 0;
    }
    public void endAccurate()    {
        pathEndVelocityConstraint = 0.1;
        pathEndTranslationalConstraint = 0.1;
        pathEndHeadingConstraint = 0.007;
        pathEndTValueConstraint = 0.995;
        pathEndTimeoutConstraint = 500;
    }
}

