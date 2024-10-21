package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flipper extends SubsystemBase {

    public final double kP = 0;
    public final double kI = 0;
    public final double kD = 0;
    public final double kF = 0;
    DcMotorEx motor;

    public PIDController pidController;

    public double currentPosition = 0;
    public double targetPosition = currentPosition;

    public Flipper(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "flipper");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        pidController.setPID(kP, kI, kD);
    }

    @Override
    public void periodic() {
        motor.setPower(pidController.calculate(motor.getCurrentPosition(), targetPosition) + Math.sin(currentPosition) * kF);
    }
}