package org.firstinspires.ftc.teamcode.auton.paths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndVelocityConstraint;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndTranslationalConstraint;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndHeadingConstraint;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndTValueConstraint;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.pathEndTimeoutConstraint;
public class AutonPathsRight {;
;
    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    private Follower follower;

    public static PathChain move2;
    public static Path movepl;
    public static Path movepr;
    public static Path move1, move1b, move1c;
    public static Path move2a, move2b, move2c, move2d, move2e, move2b2, move2t, move2d2;
    public static Path move3, move3b;
    public static Path move4, move4b;
    public static Path move5, move5b;
    public static Path move6, move6b, move6c;
    public static Path move7, move7b;
    public static Path move8;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutonPathsRight(OpMode opmode) {
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

        move1b = new Path(new BezierLine(
                new Point(31.25,96, Point.CARTESIAN),
                new Point(36.,96, Point.CARTESIAN)));
        move1b.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin

    //    move2 = follower.pathBuilder()
    //            .addPath(new BezierLine(
    //                    new Point(36,98, Point.CARTESIAN),
    //                    new Point(30,98, Point.CARTESIAN)))
    //            .addPath(new BezierLine(
    //                    new Point(30, 98, Point.CARTESIAN),
    //                    new Point(32,68, Point.CARTESIAN)))
    //             .build();



    //    move2 = new Path(new BezierCurve(
    //            new Point(36, 98, Point.CARTESIAN),
    //            new Point(32,68, Point.CARTESIAN)));
    //    move2.setConstantHeadingInterpolation(Math.toRadians(0));


        // push 1st sample to the wall
        move3 = new Path(new BezierCurve(
                new Point(62,68, Point.CARTESIAN),
                new Point(73,50, Point.CARTESIAN),
                new Point(10,54, Point.CARTESIAN)));
        move3.setConstantHeadingInterpolation(Math.toRadians(0)); // to basket


        // push 2nd sample to the wall
        move4 = new Path(new BezierCurve(
                new Point(10,56, Point.CARTESIAN),
                new Point(82,65, Point.CARTESIAN),
                new Point(82,40, Point.CARTESIAN),
                new Point(10,46, Point.CARTESIAN)));
        move4.setConstantHeadingInterpolation(Math.toRadians(0));  // back up

        // move back so specimen can be placed
        move4b = new Path(new BezierLine(
                new Point(10.,46, Point.CARTESIAN),
                new Point(27,48, Point.CARTESIAN)));
        move4b.setConstantHeadingInterpolation(Math.toRadians(0));  // to 2nd neutral specimen

        // move forward to align with specimen
        move5 = new Path(new BezierLine(
                new Point(27,48, Point.CARTESIAN),
                new Point(11,48, Point.CARTESIAN)));
        move5.setConstantHeadingInterpolation(0);// to basket

        // move to specimen
        move5b = new Path(new BezierLine(
                new Point(11,48, Point.CARTESIAN),
                new Point(8,48, Point.CARTESIAN)));
        move5b.setConstantHeadingInterpolation(0);

        // back up to get away from wall
        move6 = new Path(new BezierCurve(
                new Point(8,48, Point.CARTESIAN),
                new Point(20,48, Point.CARTESIAN)));
        move6.setConstantHeadingInterpolation(Math.toRadians(0));// to spot to hang specimin

        // move to hang 2nd specimen
        move6b = new Path(new BezierCurve(
                new Point(15,48, Point.CARTESIAN),
                new Point(31.25,99, Point.CARTESIAN)));
        move6b.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimen
        // move against the bar for 2nd specimen
        move6c = new Path(new BezierLine(
                new Point(31.25,99, Point.CARTESIAN),
                new Point(36.,99, Point.CARTESIAN)));
        move6c.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin
        // back up away from bar
        move7 = new Path(new BezierLine(
                new Point(36,99, Point.CARTESIAN),
                new Point(33,99, Point.CARTESIAN)));
        move7.setConstantHeadingInterpolation(180);// to basket

        // to park
        move7b = new Path(new BezierLine(
                new Point(33,99, Point.CARTESIAN),
                new Point(11,48, Point.CARTESIAN)));
        move7b.setConstantHeadingInterpolation(0);  // to park
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

