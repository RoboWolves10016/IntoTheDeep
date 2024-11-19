package org.firstinspires.ftc.teamcode.auton.paths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class AutonPathsLeft {

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public static Path movepl;
    public static Path movepr;
    public static Path move1, move1b, move1c;
    public static Path move1y, move1y2, move1cy;
    public static Path move2, move2a, move2b, move2c, move2d, move2e, move2b2, move2t, move2d2;
    public static Path move2ay;
    public static Path move3, move3b;
    public static Path move4, move4a;
    public static Path move5;
    public static Path move6, move6a;
    public static Path move7;
    public static Path move8;
    public static Path move12, move12b, move22a;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutonPathsLeft(OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the moves.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init()    {
        // to spot to hang specimen
        move1 = new Path(new BezierLine(
                new Point(7.25,89.25, Point.CARTESIAN),
                new Point(31.25,82, Point.CARTESIAN)));
        move1.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimin

        move1b = new Path(new BezierLine(
                new Point(31.25,82, Point.CARTESIAN),
                new Point(36.,82, Point.CARTESIAN)));
        move1b.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin

        // move to basket
        move1y = new Path(new BezierCurve(
                new Point(7.25,89.25, Point.CARTESIAN),
                new Point(51,82, Point.CARTESIAN),
                new Point(14.5,130.5, Point.CARTESIAN)));
        move1y.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45)); // to basket

        move1y2 = new Path(new BezierLine(
                new Point(7.25 - 0.72,89.25 - 0.51 , Point.CARTESIAN),
                new Point(14.5,130.5, Point.CARTESIAN)));
        move1y2.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45)); // to basket

        move1c = new Path(new BezierLine(
                new Point(34.5,78, Point.CARTESIAN),
                new Point(32,78, Point.CARTESIAN)));
        move1c.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin

        move2a = new Path(new BezierCurve(
                new Point(32.5, 82, Point.CARTESIAN),
                new Point(30,86, Point.CARTESIAN)));
        move2a.setConstantHeadingInterpolation(Math.toRadians(90));

        move2ay = new Path(new BezierCurve(
                new Point(14.5, 130, Point.CARTESIAN),
                new Point(30,87, Point.CARTESIAN)));
        move2ay.setConstantHeadingInterpolation(Math.toRadians(90));

        // to 1st neutral specimen/spike
        move2b = new Path(new BezierCurve(
             new Point(30, 87, Point.CARTESIAN),
             new Point(38,109, Point.CARTESIAN)));
        move2b.setConstantHeadingInterpolation(Math.toRadians(68));

        move2c = new Path(new BezierCurve(
                new Point(36.5, 104, Point.CARTESIAN),
                new Point(38,106, Point.CARTESIAN)));
        move2c.setConstantHeadingInterpolation(Math.toRadians(72));

        move2d = new Path(new BezierCurve(
                new Point(38, 106, Point.CARTESIAN),
                new Point(39,107.5, Point.CARTESIAN)));
        move2d.setConstantHeadingInterpolation(Math.toRadians(70));

        move2d2 = new Path(new BezierCurve(
                new Point(44, 110, Point.CARTESIAN),
                new Point(44,116.5, Point.CARTESIAN)));
        move2d2.setConstantHeadingInterpolation(Math.toRadians(70));

        move2e = new Path(new BezierCurve(
                new Point(44, 110, Point.CARTESIAN),
                new Point(44,115.5, Point.CARTESIAN)));
        move2e.setConstantHeadingInterpolation(Math.toRadians(70));

        // to basket
        move3b = new Path(new BezierLine(
                new Point(44,110, Point.CARTESIAN),
                new Point(15,130, Point.CARTESIAN)));
        move3b.setConstantHeadingInterpolation(Math.toRadians(-45)); // to basket

        // to 2nd neutral specimen/spike
        move4 = new Path(new BezierLine(
                new Point(15,130, Point.CARTESIAN),
                new Point(38.5,115, Point.CARTESIAN)));
        move4.setConstantHeadingInterpolation(Math.toRadians(68));  // to 2nd neutral specimen

        move4a = new Path(new BezierLine(
                new Point(38.5,115, Point.CARTESIAN),
                new Point(38.5,121, Point.CARTESIAN)));
        move4a.setConstantHeadingInterpolation(Math.toRadians(72));  // to 2nd neutral specimen
        // to basket
        move5 = new Path(new BezierLine(
                new Point(38.5,119, Point.CARTESIAN),
                new Point(15,130, Point.CARTESIAN)));
        move5.setConstantHeadingInterpolation(Math.toRadians(-45));// to basket

        // to 3rd neutral specimen/spike
        move6 = new Path(new BezierLine(
                new Point(15,130, Point.CARTESIAN),
                new Point(39.5,124, Point.CARTESIAN)));
        move6.setConstantHeadingInterpolation(Math.toRadians(68)); // to 3rd neutral specimen

        move6a = new Path(new BezierLine(
                new Point(39.5,124, Point.CARTESIAN),
                new Point(38.5,128, Point.CARTESIAN)));
        move6a.setConstantHeadingInterpolation(Math.toRadians(71)); // to 3rd neutral specimen

        // to basket
        move7 = new Path(new BezierLine(
                new Point(38,127, Point.CARTESIAN),
                new Point(15,130, Point.CARTESIAN)));
        move7.setConstantHeadingInterpolation(Math.toRadians(-45));// to basket

        // to park
        move8 = new Path(new BezierLine(
                new Point(15,130, Point.CARTESIAN),
                new Point(19,126, Point.CARTESIAN)));
        move8.setConstantHeadingInterpolation(Math.toRadians(0));  // to park
    }
}
