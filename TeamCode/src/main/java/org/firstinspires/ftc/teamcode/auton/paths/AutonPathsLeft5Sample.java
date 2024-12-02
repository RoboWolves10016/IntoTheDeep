package org.firstinspires.ftc.teamcode.auton.paths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class AutonPathsLeft5Sample {

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public static Path movepl;
    public static Path movepr;
    public static Path move1, move1b, move1c;
    public static Path move2a, move2b;
    public static Path move3, move3b;
    public static Path move4a, move4b;
    public static Path move5;
    public static Path move6a, move6b;
    public static Path move7;
    public static Path move8a, move8b;
    public static Path move9, move9a, move9b;
    public static Path move10;



    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutonPathsLeft5Sample(OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the moves.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init()    {
        // move to basket
        move1 = new Path(new BezierLine(
                new Point(7.25 - 0.72,89.25, Point.CARTESIAN),
                new Point(14.5,127, Point.CARTESIAN)));
        move1.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45)); // to basket
        // to 1st neutral specimen/spike
        move2a = new Path(new BezierLine(
                new Point(14.5, 130, Point.CARTESIAN),
                new Point(34,107, Point.CARTESIAN)));
        move2a.setConstantHeadingInterpolation(Math.toRadians(55));
        // to pickup 1st neutral specimen
        move2b = new Path(new BezierLine(
                new Point(34, 107, Point.CARTESIAN),
                new Point(38,113.5, Point.CARTESIAN)));
        move2b.setLinearHeadingInterpolation(Math.toRadians(55),Math.toRadians(67));

        // to basket
        move3 = new Path(new BezierLine(
                new Point(44,110, Point.CARTESIAN),
                new Point(15,130, Point.CARTESIAN))); // was 15 130
        move3.setConstantHeadingInterpolation(Math.toRadians(-45)); // to basket

        // to 2nd neutral specimen/spike
        move4a = new Path(new BezierLine(
                new Point(16, 129, Point.CARTESIAN),
                new Point(34.5,114.5, Point.CARTESIAN)));
        move4a.setConstantHeadingInterpolation(Math.toRadians(55));  // to 2nd neutral specimen

        move4b = new Path(new BezierLine(
                new Point(34,114.5, Point.CARTESIAN),
                new Point(39,122, Point.CARTESIAN)));
        move4b.setLinearHeadingInterpolation(Math.toRadians(55),Math.toRadians(70));
// to basket
        move5 = new Path(new BezierLine(
                new Point(38.5,119, Point.CARTESIAN),
                new Point(16, 129, Point.CARTESIAN)));
        move5.setConstantHeadingInterpolation(Math.toRadians(-45));// to basket

        // to 3rd neutral specimen/spike
        move6a = new Path(new BezierLine(
                new Point(16, 129, Point.CARTESIAN),
                new Point(39.5,124, Point.CARTESIAN)));
        move6a.setConstantHeadingInterpolation(Math.toRadians(68)); // to 3rd neutral specimen

        move6b = new Path(new BezierLine(
                new Point(39.5,124, Point.CARTESIAN),
                new Point(38.5,129.5, Point.CARTESIAN)));
        move6b.setConstantHeadingInterpolation(Math.toRadians(71)); // to 3rd neutral specimen

        // to basket
        move7 = new Path(new BezierLine(
                new Point(38,127, Point.CARTESIAN),
                new Point(16, 129, Point.CARTESIAN)));
        move7.setConstantHeadingInterpolation(Math.toRadians(-45));// to basket
// move to other side of field to align to 5th specimen
        move8a = new Path(new BezierCurve(
                new Point(16, 129, Point.CARTESIAN),
                new Point(33,75, Point.CARTESIAN),
                new Point(17, 51, Point.CARTESIAN)));
   //     move8a.setConstantHeadingInterpolation(Math.toRadians(-90));  // t
// move to pick up 5th specimen
        move8b = new Path(new BezierLine(
                new Point(17,51, Point.CARTESIAN),
                new Point(12, 46, Point.CARTESIAN)));
          move8b.setConstantHeadingInterpolation(Math.toRadians(-125));
// back to basket
        move9 = new Path(new BezierLine(
                new Point(12,46, Point.CARTESIAN),
                new Point(14.5, 130, Point.CARTESIAN)));
        move9.setConstantHeadingInterpolation(Math.toRadians(-45));  //

        move9a = new Path(new BezierCurve(
                new Point(12,46, Point.CARTESIAN),
                new Point(28,73, Point.CARTESIAN),
                new Point(25, 97, Point.CARTESIAN)));
        move9a.setReversed(true);
        move9a.setPathEndTimeoutConstraint(0);
        move9b = new Path(new BezierCurve(
                new Point(25,97, Point.CARTESIAN),
                new Point(14.4, 130, Point.CARTESIAN)));
        move9b.setConstantHeadingInterpolation(Math.toRadians(-45));
      //  move9b.setPathEndTimeoutConstraint(0);
        // to park
        move10 = new Path(new BezierLine(
                new Point(16, 129, Point.CARTESIAN),
                new Point(19,126, Point.CARTESIAN)));
        move10.setConstantHeadingInterpolation(Math.toRadians(0));  // to park
    }
}
