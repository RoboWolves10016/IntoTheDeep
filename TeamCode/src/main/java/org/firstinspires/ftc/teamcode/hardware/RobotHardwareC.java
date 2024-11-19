package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardwareC {
    static final double CLAWOPEN = 0.28, CLAWCLOSED = 0.49;
    static final double DUMPUP = 0.53, DUMPDOWN = 0.32;
    private double miniBasePos = 0.14, maxBasePos = 0.25;
    double intakeMax = maxBasePos, intakeMini = miniBasePos; // was 0.08, 0.146
    double transferMax = maxBasePos + 0.755, transferMini = miniBasePos + 0.495;
    double idleMax = maxBasePos + 0.705, idleMini = miniBasePos + 0.215;
    static final int ARMUP = 0, ARMDOWN = -100;
    static final double SPININ = -1, SPINOUT = 1, SPINOFF = 0;
    static final double SLIDEOUT = 0.8, SLIDEIN = 0.176;
    //static final int LIFTBAR = 3050, LIFTHOOK = 2350, LIFTBASKET = 5100, LIFTDOWN = 0, LIFTWALL = 700, CLOSECOUNTS = 5;
    static final int LIFTBAR = 1691, LIFTHOOK = 1303, LIFTBASKET = 2800, LIFTDOWN = 0, LIFTWALL = 388, CLOSECOUNTS = 20;
    static final double DUMP = 0.49, UNDUMP = 0.04; // was 0.49 0.04

    VoltageSensor battery;

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects
    public DcMotor slide = null;
    public DcMotor liftL = null;
    public DcMotor liftR = null;
    public Servo slideservo = null;
    public Servo axonMax;
    public Servo axonMini;
    public CRServo spinner;
    public Servo claw = null;
    public Servo dump = null;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime moveTimer = new ElapsedTime();

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardwareC(OpMode opmode) {myOpMode = opmode; }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        battery = myOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        // Define and Initialize Motors (note: need to use reference to actual OpMode).

        spinner = myOpMode.hardwareMap.get(CRServo.class, "spinner");
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        dump = myOpMode.hardwareMap.get(Servo.class, "dump");
        slideservo = myOpMode.hardwareMap.get(Servo.class, "slideservo");
        axonMax = myOpMode.hardwareMap.get(Servo.class, "axonmax");
        axonMini = myOpMode.hardwareMap.get(Servo.class, "axonmini");

        liftL = myOpMode.hardwareMap.get(DcMotor.class, "liftL");
        liftR = myOpMode.hardwareMap.get(DcMotor.class, "liftR");

        liftL.setDirection(DcMotor.Direction.REVERSE);
        liftR.setDirection(DcMotor.Direction.REVERSE);

        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    public void clawClosed() {
        claw.setPosition(CLAWCLOSED);
    }
    public boolean clawOpen() {
        claw.setPosition(CLAWOPEN);
        return (claw.getPosition() < 0.31);
    }
    public boolean intakePos() {
        axonMax.setPosition(intakeMax);
        axonMini.setPosition(intakeMini);
        return (true);
    }
    public boolean transferPos() {
        axonMax.setPosition(transferMax);
        axonMini.setPosition(transferMini);
        return (true);
    }
    public boolean idlePos() {
        axonMax.setPosition(idleMax);
        axonMini.setPosition(idleMini);
        return (true);
    }
    public void spinIn() {spinner.setPower(SPININ);}
    public void spinOut() {
        spinner.setPower(SPINOUT);
    }
    public void spinOff() {
        spinner.setPower(SPINOFF);
    }
    public void slideOut() {
        slideservo.setPosition(SLIDEOUT);
    }
    public void slideIn() {
        slideservo.setPosition(SLIDEIN);

    }
    public boolean liftDown() {
        liftL.setTargetPosition(LIFTDOWN);
        liftR.setTargetPosition(LIFTDOWN);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(0.9);
        liftR.setPower(0.9);
        return ((liftL.getCurrentPosition() < LIFTDOWN + 50) && (liftL.getCurrentPosition() > LIFTDOWN - 50));
    }
    public boolean liftWall() {
        liftL.setTargetPosition(LIFTWALL);
        liftR.setTargetPosition(LIFTWALL);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(0.9);
        liftR.setPower(0.9);
        return ((liftL.getCurrentPosition() < LIFTDOWN + CLOSECOUNTS) && (liftL.getCurrentPosition() > LIFTDOWN - CLOSECOUNTS));
    }
    public boolean liftBar() {
        liftL.setTargetPosition(LIFTBAR);
        liftR.setTargetPosition(LIFTBAR);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(0.9);
        liftR.setPower(0.9);

        return ((liftL.getCurrentPosition() < (LIFTBAR + CLOSECOUNTS)) && (liftL.getCurrentPosition() > (LIFTBAR - CLOSECOUNTS)));
    }
    public boolean liftHook() {
        liftL.setTargetPosition(LIFTHOOK);
        liftR.setTargetPosition(LIFTHOOK);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(0.99);
        liftR.setPower(0.99);

        return ((liftL.getCurrentPosition() < LIFTHOOK + CLOSECOUNTS) && (liftL.getCurrentPosition() > LIFTHOOK - CLOSECOUNTS));
    }
    public boolean liftBasket() {
        liftL.setTargetPosition(LIFTBASKET);
        liftR.setTargetPosition(LIFTBASKET);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(0.9);
        liftR.setPower(0.9);

        return ((liftL.getCurrentPosition() < LIFTBASKET + CLOSECOUNTS) && (liftL.getCurrentPosition() > LIFTBASKET - CLOSECOUNTS));
    }
    public void dump() {
        dump.setPosition(DUMP);
    }
    public void undump() {
        dump.setPosition(UNDUMP);
    }

} // whole file