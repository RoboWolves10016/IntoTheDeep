package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware2024;


@TeleOp(name = "Competition TeleOp Pedro", group = "Robot")
//@Disabled
public class CompetitionTeleopPedro extends OpMode {

    private Follower follower;
    RobotHardware2024  robot = new RobotHardware2024(this);

    private double rightX, leftX, leftY;
    private double left2Y, right2Y;
    private double mtrPwrPct = 1;

    double slideMaxPos = 5000, slideMinPos = 0;
    double liftMaxPos = 4950, liftMinPos = 0;
    double clawPos=0, dumpPos = 0, slidePos = 0;
    int slidewristMaxPos = 5000;
    int slidewristMinPos = 0;

    boolean lasta = false, lastb = false, lastx = false, lasty = false;
    boolean lastdpadup = false, lastdpaddown = false, lastdpadleft = false, lastdpadright = false;
    boolean lastrbump = false, lastlbump = false, lastrtrig = false, lastltrig = false;
    boolean autoUp = false, autoDown = false, autoHang = false;
    int slideCounts = 0, liftLCounts = 0, liftRCounts = 0, slidewristCounts = 0;
    double spinnerpos = 0;
    boolean autoLift = false;
    int autoHeight = 0;
    boolean recalLift = false;

    int slidewristclosedcnts = 0, slidewristopencnts = -110;
    double slidewristclosedangle = -55, slidewristopenangle = 103;

    double slidewristangle = 0, slidewristpower = 0, slidewristPoffset = 0;

    ElapsedTime wristtimer = new ElapsedTime();
    double wristextendtime = 5, swcountspersecond = 0;
    boolean wristmove = false;
    int slidewristset = 0, swsetpnt = 0;
    boolean fstmodeset = false;
    boolean slidemove = false;
    int slideset = 0;
    double slidepower = 0;

    ElapsedTime launchTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
//        intake = new Intake(hardwareMap);
//        extendo = new Extendo(hardwareMap);
//        lift = new Lift(hardwareMap);
//
//        CommandScheduler.getInstance().registerSubsystem(intake, extendo, lift);

//        AnalogInput

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

     //   telemetry.addData("Left Encoder Rotation: ", rightFront.getCurrentPosition());
    //    telemetry.addData("Right Encoder Rotation: ", rightRear.getCurrentPosition());
    //    telemetry.addData("Strafe Encoder Rotation: ", leftRear.getCurrentPosition());
        if (gamepad1.a) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, 0.5,
                    false, 0);
        } else if (gamepad1.y) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, 0.5,
                    false, 180);
        } else if (gamepad1.x) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, 0.5,
                    false, 315);
        } else {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,
                false);
        }
        follower.update();
//        CommandScheduler.getInstance().run();

// Servo check
        if (gamepad1.dpad_up) {slidePos = slidePos + 0.01; robot.slideservo.setPosition(slidePos);}
        if (gamepad1.dpad_down) {slidePos = slidePos - 0.01;robot.slideservo.setPosition(slidePos);}
        //   if (gamepad1.dpad_up) {dumpPos = dumpPos + 0.01; robot.dump.setPosition(dumpPos);}
        //   if (gamepad1.dpad_down) {dumpPos = dumpPos - 0.01;robot.dump.setPosition(dumpPos);}
// Claw
        if (gamepad2.a && !lasta) {
            if (clawPos > 0.41) {
                robot.claw.setPosition(0.28);
                clawPos = robot.claw.getPosition();
            } else {
                robot.claw.setPosition(0.49);
                clawPos = robot.claw.getPosition();
            }
        }
        lasta = gamepad2.a;

        lasty = gamepad2.y;
        if (gamepad2.y) {
            robot.dump.setPosition(0.49); // dump
        } else {
            robot.dump.setPosition(0.04); // undump
        }

// Spinner
        if (gamepad2.right_bumper) {
            robot.spinner.setPosition(0.0);
        } else if (gamepad2.right_trigger > 0.05) {
            robot.spinner.setPosition(1.0);
        } else {
            robot.spinner.setPosition(0.5);
        }
// Wrist
        if ((gamepad2.left_trigger > 0.05) && !lastltrig) {
            wristmove = true;
            if(slidewristCounts < -55) {slidewristset = slidewristclosedcnts;} else {slidewristset = slidewristopencnts;}
            wristtimer.reset();
        }
        lastltrig = (gamepad2.left_trigger > 0.05);
        slidewristCounts = robot.slidewrist.getCurrentPosition();
        slidewristangle = slidewristclosedangle-slidewristCounts*(slidewristopenangle - slidewristclosedangle)/
                (slidewristclosedcnts - slidewristopencnts);
        slidewristPoffset = 0.4*Math.sin(Math.toRadians(slidewristangle));
        slidewristpower = slidewristPoffset;
        swcountspersecond = (slidewristclosedcnts - slidewristopencnts)/wristextendtime;

        if (wristmove){
            if (slidewristset == slidewristclosedcnts) {
                if (slidewristangle > -20) {
                    if (!fstmodeset) {
                        robot.slidewrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        fstmodeset = true;
                    }
                    robot.slidewrist.setPower(0.5 + slidewristPoffset);
                } else {
                    robot.slidewrist.setTargetPosition(slidewristset);
                    robot.slidewrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.slidewrist.setPower(.99);
                    wristmove = false;
                    fstmodeset = false;
                }
            } else {
                if (slidewristangle < 80) {
                    if (!fstmodeset) {
                        robot.slidewrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        fstmodeset = true;
                    }
                    robot.slidewrist.setPower(-0.4 + slidewristPoffset);
                } else {
                    robot.slidewrist.setTargetPosition(slidewristset);
                    robot.slidewrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.slidewrist.setPower(0.99);
                    wristmove = false;
                    fstmodeset = false;
                }
            }
        } else {
            robot.slidewrist.setPower(0);
        }// wristmove
// Slide
        if (gamepad2.left_bumper&& !lastlbump) {
            if (slidePos > 0.53) {
                robot.slideservo.setPosition(0.08);
                slidePos = robot.slideservo.getPosition();
            } else {
                robot.slideservo.setPosition(0.85);
                slidePos = robot.slideservo.getPosition();
            }
        }
        lastlbump = gamepad2.left_bumper;

// Lift
        if (gamepad2.dpad_up && !lastdpadup) {
            autoLift = true;
            autoHeight = 3050;
        }
        lastdpadup = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastdpaddown) {
            autoLift = true;
            autoHeight = 2450;
        }
        lastdpaddown = gamepad2.dpad_down;

        if (gamepad2.dpad_left && !lastdpadleft) {
            autoLift = true;
            autoHeight = 700;
        }
        lastdpadleft = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !lastdpadright) {
            autoLift = true;
            autoHeight = 5100;
        }
        lastdpadright = gamepad2.dpad_right;

        if (autoLift) {
            robot.liftL.setTargetPosition(autoHeight);
            robot.liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftL.setPower(.99);
            robot.liftR.setTargetPosition(autoHeight);
            robot.liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftR.setPower(.99);
            liftLCounts = robot.liftL.getCurrentPosition();
            liftRCounts = robot.liftR.getCurrentPosition();

            if (Math.abs(liftLCounts - autoHeight) < 30) {
                autoLift = false;
            }
        }
        if ((left2Y > 0.02 && liftLCounts < liftMaxPos) || (left2Y < -0.02 && liftLCounts > liftMinPos)) {
            autoLift = false;
            robot.liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.liftR.setPower(left2Y);
            robot.liftL.setPower(left2Y);
            liftLCounts = robot.liftL.getCurrentPosition();
            liftRCounts = robot.liftR.getCurrentPosition();
        } else if (!autoLift && !recalLift) {
            robot.liftL.setTargetPosition(liftLCounts);
            robot.liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftL.setPower(.99);
            robot.liftR.setTargetPosition(liftRCounts);
            robot.liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //    robot.liftR.setPower(.99);
            robot.liftR.setPower(0.0);
            //   robot.liftL.setPower(0.0);
        }

        // change motor power percentage for greater control
        if(gamepad1.y) {mtrPwrPct = .50;} //if pad y,  50%
        if(gamepad1.b) {mtrPwrPct = .25;} //if pad b,  25%
        if(gamepad1.a) {mtrPwrPct = .99;}   //if pad a, 100%
        if(gamepad1.x) {mtrPwrPct = .75;} //if pad x,  75%

        // stick control
        leftX = gamepad1.left_stick_x;
        leftY = -gamepad1.left_stick_y;
        left2Y = -gamepad2.left_stick_y;
        right2Y = -gamepad2.right_stick_y;
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        /*****
         wheel1 = front left
         wheel2 = back left
         wheel3 = back right
         wheel4 = front right
         ******/

// Send telemetry messages to explain controls and show robot status
        telemetry.addData("Drive", "Left Stick");
        telemetry.addData("Spin", "Right Stick");
        telemetry.addData("-", "-------");

        telemetry.addData("x, y, h: ", "%.2f, %.2f, %.2f",
                follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("Drive Power, Side Power, Spin Power", "%.2f, %.2f, %.2f", leftY, leftY, rightX);

        telemetry.addData("-", "-------");
        telemetry.addData("Left2Y",  "%.2f", left2Y);
        telemetry.addData("liftL getposition",  "%d", robot.liftL.getCurrentPosition());
        telemetry.addData("liftR getposition",  "%d", robot.liftR.getCurrentPosition());
        telemetry.addData("liftL power",  "%.2f", robot.liftL.getPower());
        telemetry.addData("liftR power",  "%.2f", robot.liftR.getPower());
        telemetry.addData("auto lift / recal lift", "%b, %b", autoLift, recalLift);

        telemetry.addData("-", "-------");
        telemetry.addData("slide position", "%d", slideCounts);
        telemetry.addData("slide getposition",  "%d", robot.slide.getCurrentPosition());
        telemetry.addData("slidemove", "%b", slidemove);
        telemetry.addData("slidepwr", "%.2f", slidepower);

        telemetry.addData("-", "-------");
        telemetry.addData("left trigger",  "%.2f", gamepad2.left_trigger);
        telemetry.addData("wrist move",  "%b",wristmove);
        telemetry.addData("wrist setpoint",  "%d", slidewristset);
        telemetry.addData("wrist position", "%d", slidewristCounts);
        telemetry.addData("wrist power offset",  "%.2f", slidewristPoffset);
        telemetry.addData("wrist getposition",  "%d", robot.slidewrist.getCurrentPosition());
        telemetry.addData("wrist angle",  "%.2f", slidewristangle);

        telemetry.addData("-", "-------");
        telemetry.addData("Right bumper 2",  "%b",gamepad2.right_bumper);
        telemetry.addData("Right trigger 2", "%.2f", gamepad2.right_trigger);
        telemetry.addData("Spin Position",  "%.2f", robot.spinner.getPosition());

        telemetry.addData("claw position: ",  "%.2f", clawPos);
        telemetry.addData("dump position: ",  "%.2f", robot.dump.getPosition());
        telemetry.addData("slide position: ",  "%.2f", robot.slideservo.getPosition());

        //telemetry.addData("lift down ", "%b", robot.liftdown.isPressed());
        if (robot.liftdown.isPressed()) {
            telemetry.addData("Touch Sensor", "Is Pressed");
        } else {
            telemetry.addData("Touch Sensor", "Is Not Pressed");
        }

        telemetry.update();

        // Pace this loop so hands move at a reasonable speed.
    //    sleep(50);


        telemetry.update();
    }
}
