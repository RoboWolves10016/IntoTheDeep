package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware2024;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="OnBot Teleop", group="Robot")

public class OnBotTeleop extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware2024  robot = new RobotHardware2024(this);

    private double rightX, leftX, leftY;
    private double left2Y, right2Y;
    private double mtrPwrPct = 1;

    double slideMaxPos = 5000, slideMinPos = 0;
    double liftMaxPos = 4950, liftMinPos = 0;
    double clawPos=0, dumpPos = 0;
    int slidewristMaxPos = 5000;
    int slidewristMinPos = 0;

    boolean lasta = false, lastb = false, lastx = false, lasty = false;
    boolean lastrbump = false, lastlbump = false, lastrtrig = false, lastltrig = false;
    boolean autoUp = false, autoDown = false, autoHang = false;
    int slideCounts = 0, liftLCounts = 0, liftRCounts = 0, slidewristCounts = 0;
    double spinnerpos = 0;

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

    @Override
    public void runOpMode() {

        ElapsedTime launchTimer = new ElapsedTime();
        ElapsedTime autoTimer = new ElapsedTime();

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
// Servo check
            if (gamepad2.dpad_up) {dumpPos = dumpPos + 0.01; robot.dump.setPosition(dumpPos);}
            if (gamepad2.dpad_down) {dumpPos = dumpPos - 0.01;robot.dump.setPosition(dumpPos);}
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
// Dump
            if (gamepad2.y && !lasty) {
                if (dumpPos > 0.53) {
                    robot.dump.setPosition(0.32);
                    dumpPos = robot.dump.getPosition();
                } else {
                    robot.dump.setPosition(0.65);
                    dumpPos = robot.dump.getPosition();
                }
            }
            lasty = gamepad2.y;

// Spinner
            //    control the spinner with "right bumper" button on gamepad 2
            //   press to turn on then again to turn off
            //   reverse with right trigger - once on then again to turn off
            if (gamepad2.right_bumper && !lastrbump) {
                if (robot.spinner.getPosition() < 0.4) {
                    robot.spinner.setPosition(0.5);
                } else {
                    robot.spinner.setPosition(0);
                }
            }
            lastrbump = gamepad2.right_bumper;

            if ((gamepad2.right_trigger > 0.05) && !lastrtrig) {
                if (robot.spinner.getPosition() > 0.6) {
                    robot.spinner.setPosition(0.5);
                } else {
                    robot.spinner.setPosition(1);
                }
            }
            lastrtrig = (gamepad2.right_trigger > 0.05);
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
            slideCounts = robot.slide.getCurrentPosition();
/*            if( right2Y > 0.05) {
                robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slide.setPower(right2Y);
            } else if( right2Y < -0.1) {
                robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slide.setPower(right2Y);
            } else  {
                if (slideCounts < 20 ){
                    robot.slide.setPower(0.5);
                } else {
                    robot.slide.setPower(-0.5);
                }
            }
            slideCounts = robot.slide.getCurrentPosition();
  */          if ((gamepad2.left_bumper ) && !lastlbump) {
                slidemove = true;
                if(slideCounts < 20) {slideset = 40;} else {slideset = 0;}
            }
            lastlbump = (gamepad2.left_bumper);

            if (slidemove){
                if (slideset == 0) {
                    if (slideCounts > 5) {
                        slidepower = -0.99;
                    } else {slidemove = false;}
                } else {
                    if (slideCounts <38) {
                        slidepower = 0.99;
                    } else {slidemove = false;}
                }
            } else {
                if (slideCounts < 20 ){
                    slidepower = -0.02;
                } else {
                    slidepower = 0.02;
                }
            }// slidemove
            // robot.slide.setPower(slidepower);

// Lift
            if ((left2Y > 0.02 && liftLCounts < liftMaxPos) || (left2Y < -0.02 && liftLCounts > liftMinPos)) {
                robot.liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //   robot.liftR.setPower(left2Y);
                robot.liftL.setPower(left2Y);
                liftLCounts = robot.liftL.getCurrentPosition();
                liftRCounts = robot.liftR.getCurrentPosition();
            } else  {
                robot.liftL.setTargetPosition(liftLCounts);
                robot.liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftL.setPower(.99);
                //   robot.liftR.setTargetPosition(liftRCounts);
                //    robot.liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //     robot.liftR.setPower(.99);
                //     robot.liftR.setPower(0.0);
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

            // Spin left or right by right stick
            if(gamepad1.right_stick_x != 0 || gamepad1.right_trigger > 0|| gamepad1.left_trigger > 0) {
                if(gamepad1.right_stick_x > 0) {
                    rightX = (gamepad1.right_stick_x * 0.75);
                }
                if(gamepad1.right_stick_x < 0) {
                    rightX = (gamepad1.right_stick_x * 0.75);
                }
                if(gamepad1.right_trigger > 0) {
                    rightX = 0.5;
                }
                if(gamepad1.left_trigger > 0) {
                    rightX = -0.5;
                }
            }
            else rightX = 0;
            leftX = leftX * mtrPwrPct;
            rightX = rightX * mtrPwrPct;

            leftY = leftY * mtrPwrPct;

            // if the sticks are 0 stop
            if((Math.abs(leftY) + Math.abs(rightX) + Math.abs(leftX)) < 0.01) {
                robot.setDrivePower(0,0,0,0);
            } else {
                robot.driveRobot(leftY, leftX , rightX + (0.05*leftY));
            }

// Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Spin", "Right Stick");
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", leftY);
            telemetry.addData("Side Power",  "%.2f", leftX);
            telemetry.addData("Spin Power",  "%.2f", rightX);

            telemetry.addData("-", "-------");
            telemetry.addData("Left2Y",  "%.2f", left2Y);
            telemetry.addData("liftL getposition",  "%d", robot.liftL.getCurrentPosition());
            telemetry.addData("liftR getposition",  "%d", robot.liftR.getCurrentPosition());

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
            telemetry.addData("dump position: ",  "%.2f", dumpPos);

            //telemetry.addData("lift down ", "%b", robot.liftdown.isPressed());
            if (robot.liftdown.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }

            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        } // while (opModeIsActive())
    } // runOpMode()

} // whole file
