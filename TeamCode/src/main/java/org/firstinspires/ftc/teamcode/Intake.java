package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SimpleSubsystem {

    public final double OUT_POSITION = 0;
    public final double IN_POSITION = 0;
    public final double IN_POWER = 1;
    public final double OUT_POWER = -1;
    public final double kP = 0;
    public final double kI = 0;
    public final double kD = 0;
    public final double kG = 0;
    DcMotorEx flipper;
    CRServo spinner;
    double spinnerPower = 0;

    public PIDController pidController;

    public double currentPosition = 0;
    public double targetPosition = currentPosition;

    private static Intake instance;

    public static Intake getInstance(HardwareMap hwMap) {
        if(instance == null) {
            instance = new Intake(hwMap);
        }
        return instance;
    }

    private Intake(HardwareMap hwMap) {
        spinner = hwMap.get(CRServo.class, "spinner");

        flipper = hwMap.get(DcMotorEx.class, "flipper");
        flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Positive direction is out from robot
        flipper.setDirection(DcMotorSimple.Direction.FORWARD);

        pidController.setPID(kP, kI, kD);
    }

    @Override
    public void periodic() {
        currentPosition = flipper.getCurrentPosition() * 360 / 288d;
        double pid = pidController.calculate(currentPosition, targetPosition);
        double ff = Math.cos(Math.toRadians(currentPosition)) * kG;
        flipper.setPower(pid + ff);

        spinner.setPower(spinnerPower);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Flipper Pos Deg: ", currentPosition);
        telemetry.addData("Flipper Target Deg: ", targetPosition);
        telemetry.update();
    }

    public void flipOut() {
        targetPosition = OUT_POSITION;
    }

    public void flipIn() {
        targetPosition = IN_POSITION;
    }

    public void intake() {
        spinnerPower = IN_POWER;
    }

    public void exhaust() {
        spinnerPower = OUT_POWER;
    }

    public void stopSpinner() {
        spinnerPower = 0;
    }
}