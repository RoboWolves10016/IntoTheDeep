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

public class AutonPathsRight4H {;
;
    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    private Follower follower;

    public static Path move1, move1b;
    public static PathChain move2a;
    public static Path move2b;
    public static Path move3a, move3b;
    public static PathChain move4a;
    public static Path move4b, move4c;
    public static Path move5a, move5b, move5c;
    public static PathChain move6a;
    public static Path move6b, move6c;
    public static Path move7a, move7b, move7c;
    public static Path move8a, move8b, move8c;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutonPathsRight4H(OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the moves.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init(Follower follower)    {
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

        move2a = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(36,96, Point.CARTESIAN),
                        new Point(25,99, Point.CARTESIAN),
                        new Point(19,80, Point.CARTESIAN),
                        new Point(31,73, Point.CARTESIAN)))
                .addPath(new BezierLine(
                        new Point(32, 73, Point.CARTESIAN),
                        new Point(58,65, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(58,65, Point.CARTESIAN),
                        new Point(58,45, Point.CARTESIAN),
                        new Point(25,58, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(25,58, Point.CARTESIAN),
                        new Point(42,58, Point.CARTESIAN),
                        new Point(58,58, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(58,58, Point.CARTESIAN),
                        new Point(58,34, Point.CARTESIAN),
                        new Point(25,46, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(25,46, Point.CARTESIAN),
                        new Point(32,59, Point.CARTESIAN),
                        new Point(12,60.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(12,59, Point.CARTESIAN),
                        new Point(8,59, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.5)
                .build();
// push 2 samples into the zone

        move2b = new Path(new BezierLine(
                new Point(11,60, Point.CARTESIAN),
                new Point(8,60, Point.CARTESIAN)));
        move2b.setConstantHeadingInterpolation(Math.toRadians(0));
// move to the bar with 2nd specimen
        move3a = new Path(new BezierCurve(
                new Point(15, 55, Point.CARTESIAN),
                new Point(31.25,99, Point.CARTESIAN)));
        move3a.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimen
        move3a.setPathEndTimeoutConstraint(0);
// move against the bar to hang 2nd specimen
        move3b = new Path(new BezierLine(
                new Point(31.25,99, Point.CARTESIAN),
                new Point(36.,99, Point.CARTESIAN)));
        move3b.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin
// back away from bar and turn to head towards 3rd specimen
        move4a = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(36, 99, Point.CARTESIAN),
                        new Point(33,99, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Point(33,99, Point.CARTESIAN),
                        new Point(11,72, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.3)
                .build();
/*        move4a = new Path(new BezierLine(
                new Point(36,99, Point.CARTESIAN),
                new Point(33,99, Point.CARTESIAN)));
        move4a.setConstantHeadingInterpolation(Math.toRadians(180));
        move4a.setPathEndTimeoutConstraint(0); // back away from bar
*/// align to 3rd specimen
        move4b = new Path(new BezierLine(
                new Point(33,99, Point.CARTESIAN),
                new Point(11,72, Point.CARTESIAN)));
        move4b.setConstantHeadingInterpolation(Math.toRadians(0));  //
// pick up 3rd specimen
        move4c = new Path(new BezierLine(
                new Point(11,72, Point.CARTESIAN),
                new Point(8,71, Point.CARTESIAN)));
        move4c.setConstantHeadingInterpolation(Math.toRadians(0));
// move to hang 3rd specimen
        move5b = new Path(new BezierCurve(
                new Point(20,80, Point.CARTESIAN),
                new Point(31.25,101, Point.CARTESIAN)));
        move5b.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimen
        move5b.setPathEndTimeoutConstraint(0);
// move against the bar to hang 3rd specimen
        move5c = new Path(new BezierLine(
                new Point(31.25,101, Point.CARTESIAN),
                new Point(36.,101, Point.CARTESIAN)));
        move5c.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin
// back away from bar and turn to head towards 4th specimen
        move6a = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(36, 101, Point.CARTESIAN),
                        new Point(33,101, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Point(33,101, Point.CARTESIAN),
                        new Point(11,72, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.3)
                .build();
/*        move6a = new Path(new BezierLine(
                new Point(36,101, Point.CARTESIAN),
                new Point(33,101, Point.CARTESIAN)));
        move6a.setConstantHeadingInterpolation(Math.toRadians(180));
        move6a.setPathEndTimeoutConstraint(0); // back away from bar
*/// align to 4th specimen
        move6b = new Path(new BezierLine(
                new Point(33,101, Point.CARTESIAN),
                new Point(11,72, Point.CARTESIAN)));
        move6b.setConstantHeadingInterpolation(Math.toRadians(0));  //
// pick up 4th specimen
        move6c = new Path(new BezierLine(
                new Point(11,72, Point.CARTESIAN),
                new Point(8,71, Point.CARTESIAN)));
        move6c.setConstantHeadingInterpolation(Math.toRadians(0));  // to park
// move to hang 4th specimen
        move7b = new Path(new BezierCurve(
                new Point(20,80, Point.CARTESIAN),
                new Point(31.25,103, Point.CARTESIAN)));
        move7b.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimen
        move7b.setPathEndTimeoutConstraint(0);
// move against the bar to hang 4th specimen
        move7c = new Path(new BezierLine(
                new Point(31.25,103, Point.CARTESIAN),
                new Point(36.,103, Point.CARTESIAN)));
        move7c.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin
// park
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

