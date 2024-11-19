package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware2024;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "Competition TeleOp Pedro", group = "Robot")
//@Disabled
public class CompetitionTeleopPedro2 extends OpMode {

    private Follower follower;
    private final Pose resetPose = new Pose(0, 0, Math.toRadians(0));
    RobotHardware2024 robot = new RobotHardware2024(this);
    DcMotorEx liftL, liftR;
    private double rightX, leftX, leftY;
    private double left2Y, right2Y;
    private double mtrPwrPct = 1;

    double slideMaxPos = 5000, slideMinPos = 0;
    double liftMaxPos = 2800, liftMinPos = 0;
    double clawPos=0, dumpPos = 0, slidePos = 0;
    int slidewristMaxPos = 5000;
    int slidewristMinPos = 0;

    boolean lasta = false, lastb = false, lastx = false, lasty = false;
    boolean lastdpadup = false, lastdpaddown = false, lastdpadleft = false, lastdpadright = false;
    boolean lastrbump = false, lastlbump = false, lastrtrig = false, lastltrig = false;
    boolean last2Back = false;
    boolean autoUp = false, autoDown = false, autoHang = false;
    int slideCounts = 0, liftLCounts = 0, liftRCounts = 0, slidewristCounts = 0;
    double spinnerpos = 0;
    boolean autoLift = false;
    int autoHeight = 0;
    boolean recalLift = false;

    private double miniBasePos = 0.14, maxBasePos = 0.25;
    private double miniTarget = miniBasePos + 0.215;
    private double maxTarget = maxBasePos + 0.705;
    private double slideTarget = 0.176;
    private double spinnerSpeed = 0;
    private boolean gotoPre = false, inIntake = false;

    private boolean inTransfer = false;

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
    boolean fieldOriented = false, lastLBump = false;
    double driveSpeed = 1;
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
        liftL = (DcMotorEx) robot.liftL;
        liftR = (DcMotorEx) robot.liftR;

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        //   telemetry.addData("Left Encoder Rotation: ", rightFront.getCurrentPosition());
        //    telemetry.addData("Right Encoder Rotation: ", rightRear.getCurrentPosition());
        //    telemetry.addData("Strafe Encoder Rotation: ", leftRear.getCurrentPosition());

        // Old toggle robot centric drive
//        if (gamepad1.left_bumper && !lastLBump) {
//            if (fieldcoord) {
//                fieldcoord = false;
//            } else {
//                fieldcoord = true;
//            }
//        }
//        lastLBump = gamepad1.left_bumper;

        fieldOriented = gamepad1.left_trigger < 0.1;

        if(gamepad1.right_trigger > 0.1) {
            driveSpeed = 0.25;
        } else {
            driveSpeed = 1.0;
        }

        if (gamepad1.a) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, 0.5,
                    false, 0);
        } else if (gamepad1.y) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, 0.5,
                    false, 180);
        } else if (gamepad1.x) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, 0.5,
                    false, 315);
        } else {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, -gamepad1.right_stick_x * driveSpeed/2,
                    !fieldOriented);
        }
        if (gamepad1.back) {
            follower.setPose(resetPose);
        }
        follower.update();
//        CommandScheduler.getInstance().run();

// Servo check
     //   if (gamepad1.dpad_up) {slidePos = slidePos + 0.01; robot.slideservo.setPosition(slidePos);}
    //    if (gamepad1.dpad_down) {slidePos = slidePos - 0.01;robot.slideservo.setPosition(slidePos);}
           if (gamepad1.dpad_up) {dumpPos = dumpPos + 0.01; robot.dump.setPosition(dumpPos);}
           if (gamepad1.dpad_down) {dumpPos = dumpPos - 0.01;robot.dump.setPosition(dumpPos);}
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
            checkIntakeCollision();
        } else {
           robot.dump.setPosition(0.04); // undump
        }

// Eater and puke
        // recal
        if (gamepad2.back) {
            spinnerSpeed = 0;
            if (gamepad2.right_stick_y > 0.02) {
                miniTarget = miniTarget - 0.0004;
            }
            if (gamepad2.right_stick_y < -0.02) {
                miniTarget = miniTarget + 0.0004;
            }
            if (gamepad2.right_stick_x > 0.02) {
                maxTarget = maxTarget - 0.0004;
            }
            if (gamepad2.right_stick_x < -0.02) {
                maxTarget = maxTarget + 0.0004;
            }
        } else {
            if (last2Back && miniTarget < 0.3 && maxTarget < 0.3) {
                miniBasePos = miniTarget;
                maxBasePos = maxTarget;
            }
        }
        last2Back = gamepad2.back;
        // PreIntake
        if ((gamepad2.right_bumper || gotoPre) && !gamepad2.back) {
            maxTarget = maxBasePos + 0.165; // was 0.23
            miniTarget = miniBasePos + 0.015; // was 0.05 .146
            gotoPre = false;
            inTransfer = false;
        }

        // Intake
        if (gamepad2.right_trigger > 0.02) {
            maxTarget = maxBasePos; // was 0.21
            miniTarget = miniBasePos; // was 0.144  0.18 0.155  175
            if(!gamepad2.b) {
                spinnerSpeed = -1;
            }
            inIntake = true;
            inTransfer = false;
        } else if (inIntake) {
            gotoPre = true;
            inIntake = false;
        }
        // Transfer
        if (gamepad2.left_bumper) {
            maxTarget = maxBasePos + 0.755; // was .083
            miniTarget = miniBasePos + 0.495; // 0.67
            slideTarget = 0.176; // make sure slides are all the way in
            inTransfer = true;
        }
        // Idle
        if (gamepad2.left_trigger > 0.02) {
            maxTarget = maxBasePos + 0.705; // 0.70
            miniTarget = miniBasePos + 0.215; // .45
            inTransfer = false;
        }

        // Puke
        if(gamepad2.b) {
            spinnerSpeed = 1;
        }

        if(!gamepad2.b && gamepad2.right_trigger <= 0.02) {
            spinnerSpeed = 0;
        }
        robot.axonMax.setPosition(maxTarget);
        robot.axonMini.setPosition(miniTarget);
        robot.spinner.setPower(spinnerSpeed);

// Slide
        if (gamepad2.right_stick_y < -0.02 && slideTarget <= 0.8 && !gamepad2.back ) {
            slideTarget = slideTarget + 0.03;
        }
        if (gamepad2.right_stick_y > 0.02 && slideTarget >= 0.176 && !gamepad2.back) {
            slideTarget = slideTarget - 0.03;
        }
        robot.slideservo.setPosition(slideTarget);
     /*   if (gamepad2.left_bumper&& !lastlbump) {
            if (slidePos > 0.53) {
                robot.slideservo.setPosition(0.08);
                slidePos = robot.slideservo.getPosition();
            } else {
                robot.slideservo.setPosition(0.85);
                slidePos = robot.slideservo.getPosition();
            }
        }
        lastlbump = gamepad2.left_bumper;
*/
// Lift
        if (gamepad2.right_stick_button) {
            recalLift = true;
            if (robot.liftdown.isPressed()) {
                robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.liftL.setPower(-0.8);
            }
        } else if (recalLift) {
            robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            recalLift = false;
        }
        if (gamepad2.dpad_up && !lastdpadup) { // Lift Bar
            autoLift = true;
            autoHeight = 1691;
        }
        lastdpadup = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastdpaddown) { // Lift Hook
            autoLift = true;
            autoHeight = 1330;
        }
        lastdpaddown = gamepad2.dpad_down;

        if (gamepad2.dpad_left && !lastdpadleft) { // Lift Wall
            autoLift = true;
            autoHeight = 388;
        }
        lastdpadleft = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !lastdpadright) { // Lift Basket
            autoLift = true;
            autoHeight = 2800;
            checkIntakeCollision();
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
            // never set the hold target to a value below 0
            liftLCounts = Math.max(robot.liftL.getCurrentPosition(), 0);
            liftRCounts = Math.max(robot.liftR.getCurrentPosition(), 0);
            checkIntakeCollision();
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

        telemetry.addData("x, y, h: ", "%.2f, %.2f, %.2f",
                follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("-", "-------");
        telemetry.addData("mini , max: ", "%.2f, %.2f", miniTarget, maxTarget);
        telemetry.addData("-", "-------");
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
//        telemetry.addData("slide getposition",  "%d", robot.slide.getCurrentPosition());
        telemetry.addData("slidemove", "%b", slidemove);
        telemetry.addData("slidepwr", "%.2f", slidepower);

        telemetry.addData("-", "-------");
        telemetry.addData("left trigger",  "%.2f", gamepad2.left_trigger);
        telemetry.addData("wrist move",  "%b",wristmove);
        telemetry.addData("wrist setpoint",  "%d", slidewristset);
        telemetry.addData("wrist position", "%d", slidewristCounts);
        telemetry.addData("wrist power offset",  "%.2f", slidewristPoffset);
        //   telemetry.addData("wrist getposition",  "%d", robot.slidewrist.getCurrentPosition());
        telemetry.addData("wrist angle",  "%.2f", slidewristangle);

        telemetry.addData("-", "-------");
        telemetry.addData("Right bumper 2",  "%b",gamepad2.right_bumper);
        telemetry.addData("Right trigger 2", "%.2f", gamepad2.right_trigger);
        //   telemetry.addData("Spin Position",  "%.2f", robot.spinner.getPosition());

        telemetry.addData("claw position: ",  "%.2f", clawPos);
        telemetry.addData("dump position: ",  "%.2f", robot.dump.getPosition());
        telemetry.addData("slide position: ",  "%.2f", robot.slideservo.getPosition());

        //telemetry.addData("lift down ", "%b", robot.liftdown.isPressed());
        if (robot.liftdown.isPressed()) {
            telemetry.addData("Touch Sensor", "Is Pressed");
        } else {
            telemetry.addData("Touch Sensor", "Is Not Pressed");
        }
//        liftL = (DcMotorEx) robot.liftL;
//        liftR = (DcMotorEx) robot.liftR;
        telemetry.addData("Lift Motor Current L/R ",  "%.2f / %.2f",
                liftL.getCurrent(CurrentUnit.MILLIAMPS), liftR.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();

        // Pace this loop so hands move at a reasonable speed.
        //    sleep(50);
    }

    private void checkIntakeCollision() {
        if (inTransfer) {
            maxTarget = maxBasePos + 0.705; // 0.70
            miniTarget = miniBasePos + 0.215; // .45        }
        }
    }
}
