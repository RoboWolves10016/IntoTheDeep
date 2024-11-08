package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RobotHardware {

    static final int CLAWOPEN = 0, CLAWCLOSED = 0;
    static final int DUMPUP = 0, DUMPDOWN = 0;
    static final int ARMUP = 0, ARMDOWN = 0;
    static final int SPININ = 0, SPINOUT = 0, SPINOFF = 0;
    static final int SLIDEOUT = 0, SLIDEIN = 0;
    static final int LIFTBAR = 0, LIFTHOOK = 0, LIFTBASKET = 0, LIFTDOWN = 0, CLOSECOUNTS = 5;;
    static final double DUMP = 0.65, UNDUMP = 0.32;

    VoltageSensor battery;

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects
    public DcMotor slide = null;
    public DcMotor liftL = null;
    public DcMotor liftR = null;
    public DcMotor slidewrist = null;
    public Servo spinner = null;
    public Servo claw = null;
    public Servo dump = null;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime moveTimer = new ElapsedTime();

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(OpMode opmode) {myOpMode = opmode; }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        battery = myOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        // Define and Initialize Motors (note: need to use reference to actual OpMode).

        spinner = myOpMode.hardwareMap.get(Servo.class, "spinner");
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        dump = myOpMode.hardwareMap.get(Servo.class, "dump");

        slide  = myOpMode.hardwareMap.get(DcMotor.class, "slide");
        slidewrist  = myOpMode.hardwareMap.get(DcMotor.class, "slidewrist");
        liftL = myOpMode.hardwareMap.get(DcMotor.class, "liftL");
        liftR = myOpMode.hardwareMap.get(DcMotor.class, "liftR");

        slidewrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setDirection(DcMotor.Direction.FORWARD);
        //lift.setDirection(DcMotor.Direction.REVERSE);
        slidewrist.setDirection(DcMotor.Direction.REVERSE);


        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidewrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidewrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    public boolean clawClosed() {
        claw.setPosition(CLAWCLOSED);
        return claw.getPosition() > CLAWCLOSED - CLOSECOUNTS;
    }
    public void clawOpen() {

    }
    public void armDown() {

    }
    public void armUp() {

    }
    public void spinIn() {

    }
    public void spinOut() {

    }
    public void slideOut() {

    }
    public void slideIn() {

    }
    public boolean liftDown() {
        liftL.setTargetPosition(LIFTDOWN);
        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftL.setPower(0.99);
        return liftL.getCurrentPosition() < 5;
    }
    public boolean liftBar() {

        return true;
    }
    public boolean liftHook() {

        return true;
    }
    public boolean liftBasket() {

        return true;
    }
    public void dump() {

    }
    public void undump() {

    }

} // whole file