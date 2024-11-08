package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift extends SimpleSubsystem {

    private static final int MAX_HEIGHT_TICKS = 1;
    private static final double MAX_HEIGHT_INCHES = 0;

    private static final double LOW_BASKET_POSITION = 28;
    private static final double HIGH_BASKET_POSITION = 40;
    private static final double PRE_LOW_CHAMBER_POSITION = 20;
    private static final double LOW_CHAMBER_POSITION = 20;
    private static final double PRE_HIGH_CHAMBER_POSITION = 35;
    private static final double HIGH_CHAMBER_POSITION = 33;

    private final double INCHES_PER_TICK = MAX_HEIGHT_INCHES / MAX_HEIGHT_TICKS;

    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kF = 0;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private final PIDController leftPidController;
    private final PIDController rightPidController;

    private double leftPosition = 0;
    private double rightPosition = 0;
    private double target = 0;

    public Lift(HardwareMap hwMap) {

        leftMotor = hwMap.get(DcMotorEx.class, "rightLift");
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor = hwMap.get(DcMotorEx.class, "leftLift");
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftPidController = new PIDController(kP, kI, kD);
        rightPidController = new PIDController(kP, kI, kD);
    }

    @Override
    public void periodic() {
        leftPosition = leftMotor.getCurrentPosition() * INCHES_PER_TICK;
        rightPosition = rightMotor.getCurrentPosition() * INCHES_PER_TICK;
        double leftPid = leftPidController.calculate(leftPosition, target);
        double rightPid = rightPidController.calculate(rightPosition, target);
        leftMotor.setPower(leftPid + kF);
        rightMotor.setPower(rightPid + kF);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Left Lift Pos: ", leftPosition);
        telemetry.addData("Right Lift Pos: ", rightPosition);
        telemetry.addData("Lift Target: ", target);
        telemetry.update();
    }

    public boolean atPosition() {
        return Math.abs(leftPosition - target) < 1 && Math.abs(rightPosition - target) < 1;
    }

}