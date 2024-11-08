package org.firstinspires.ftc.teamcode.auton.paths;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class AutonPaths1A {

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public static Path movepl;
    public static Path movepr;
    public static Path move1, move1b, move1c;
    public static Path move2, move2a, move2b, move2b2, move2t;
    public static Path move3;
    public static Path move4;
    public static Path move5;
    public static Path move6;
    public static Path move7;
    public static Path move8;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public AutonPaths1A(OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the moves.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init()    {
        // to spot to hang specimen
        movepl = new Path(new BezierLine(
                new Point(9.75,100, Point.CARTESIAN),
                new Point(50,100, Point.CARTESIAN)));
        movepl.setConstantHeadingInterpolation(Math.toRadians(0)); // to spot to hang specimin

        movepr = new Path(new BezierLine(
                new Point(9.75,60, Point.CARTESIAN),
                new Point(49.75,60, Point.CARTESIAN)));
        movepr.setConstantHeadingInterpolation(90); // to spot to hang specimin

        move1 = new Path(new BezierLine(
                new Point(7.25,87.75, Point.CARTESIAN),
                new Point(32.25,78, Point.CARTESIAN)));
        move1.setConstantHeadingInterpolation(Math.toRadians(180));// to spot to hang specimin

        move1b = new Path(new BezierLine(
                new Point(32.25,78, Point.CARTESIAN),
                new Point(36.5,78, Point.CARTESIAN)));
        move1b.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin

        move1c = new Path(new BezierLine(
                new Point(36.5,78, Point.CARTESIAN),
                new Point(32,78, Point.CARTESIAN)));
        move1b.setConstantHeadingInterpolation(Math.toRadians(180)); // to spot to hang specimin

     //   move2a = new Path(new BezierCurve(
     //           new Point(32.5, 78, Point.CARTESIAN),
     //           new Point(30,86, Point.CARTESIAN)));
     //   move2a.setConstantHeadingInterpolation(Math.toRadians(90));

        move2a = new Path(new BezierCurve(
                new Point(32.5, 78, Point.CARTESIAN),
                new Point(35,102, Point.CARTESIAN),
                new Point(40,91, Point.CARTESIAN),
                new Point(43,103, Point.CARTESIAN)));
        move2a.setConstantHeadingInterpolation(Math.toRadians(90));

        // to 1st neutral specimen
  //      move2b = new Path(new BezierCurve(
  //              new Point(30, 86, Point.CARTESIAN),
  //              new Point(39,107, Point.CARTESIAN)));
  //      move2b.setConstantHeadingInterpolation(Math.toRadians(72));
    //    move2b = new Path(new BezierCurve(
    //            new Point(30, 86, Point.CARTESIAN),
    //            new Point(27.1,94.6, Point.CARTESIAN),
    //            new Point(45,95.5, Point.CARTESIAN),
    //            new Point(44,107, Point.CARTESIAN)));
    //    move2b.setConstantHeadingInterpolation(Math.toRadians(90));


        // to basket
        move3 = new Path(new BezierLine(
                new Point(39,107, Point.CARTESIAN),
                new Point(15,131, Point.CARTESIAN)));
        move3.setConstantHeadingInterpolation(Math.toRadians(-45)); // to basket

        // to 2nd neutral specimen
        move4 = new Path(new BezierLine(
                new Point(14.,129, Point.CARTESIAN),
                new Point(56,105, Point.CARTESIAN)));
        move4.setConstantHeadingInterpolation(Math.toRadians(-45));  // to 2nd neutral specimen

        // to basket
        move5 = new Path(new BezierLine(
                new Point(22.25,130.35, Point.CARTESIAN),
                new Point(17.25,124.25, Point.CARTESIAN)));
        move5.setConstantHeadingInterpolation(-.78);// to basket

        // to 3rd neutral specimen
        move6 = new Path(new BezierLine(
                new Point(17.25,124.25, Point.CARTESIAN),
                new Point(22.25,133.25, Point.CARTESIAN)));
        move6.setConstantHeadingInterpolation(-2.26); // to 3rd neutral specimen

        // to basket
        move7 = new Path(new BezierLine(
                new Point(22.25,133.25, Point.CARTESIAN),
                new Point(17.25,124.25, Point.CARTESIAN)));
        move7.setConstantHeadingInterpolation(-.78);// to basket

        // to park
        move8 = new Path(new BezierLine(
                new Point(17.25,124.25, Point.CARTESIAN),
                new Point(62.25,101.25, Point.CARTESIAN)));
        move8.setConstantHeadingInterpolation(-.74);  // to park
    }
}
