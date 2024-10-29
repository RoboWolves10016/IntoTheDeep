package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Extendo extends SimpleSubsystem {

    public static final double IN_POSITION = 0;
    public static final double OUT_POSITION = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    private final DcMotorEx motor;

    private final PIDController pidController;

    private double currentPosition = 0;
    private double targetPosition = 0;

    public Extendo(HardwareMap hwMap) {

        motor = hwMap.get(DcMotorEx.class, "extendo");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Positive direction is out from robot
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        pidController = new PIDController(kP, kI, kD);
    }

    public boolean atPosition() {
        return Math.abs(currentPosition - targetPosition) < 1;
    }

    @Override
    public void periodic() {
        currentPosition = motor.getCurrentPosition() * 360 / 288d;
        double pid = pidController.calculate(currentPosition, targetPosition);
        motor.setPower(pid);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Extendo Pos Deg: ", currentPosition);
        telemetry.addData("Extendo Pos Deg: ", targetPosition);
        telemetry.update();
    }

    public void setTargetPosition(double target) {
        targetPosition = target;
    }

}