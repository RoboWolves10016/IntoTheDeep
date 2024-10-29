package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.Set;
@Config
public class Intake extends SimpleSubsystem {

    public static final double IN_POSITION = -27;
    public static final double OUT_POSITION = 0;
    public static final double IN_POWER = 1;
    public static final double OUT_POWER = -1;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;

    private DcMotorEx flipper;
    private CRServo spinner;

    double spinnerPower = 0;

    private PIDController pidController;

    private double currentPosition = IN_POSITION;
    private double targetPosition = currentPosition;

    public Intake(HardwareMap hwMap) {
        spinner = hwMap.get(CRServo.class, "spinner");

        flipper = hwMap.get(DcMotorEx.class, "flipper");
        flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Positive direction is out from robot
        flipper.setDirection(DcMotorSimple.Direction.FORWARD);

        pidController = new PIDController(kP, kI, kD);
    }

    @Override
    public void periodic() {
        currentPosition = -27 + flipper.getCurrentPosition() * 360 / 288d;
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

    public boolean atPosition() {
        return Math.abs(currentPosition - targetPosition) < 1;
    }

    public Command intakeCommand(Intake intake) {
        return new CommandBase() {
            @Override
            public void initialize() {
                addRequirements(intake);
                intake.flipOut();
            }

            @Override
            public void end(boolean interrupted) {
                intake.intake();
            }

            @Override
            public boolean isFinished() {
                return intake.atPosition();
            }
        };
    }

    public Command retractCommand(Intake intake) {
        return new CommandBase() {
            @Override
            public void initialize() {
                addRequirements(intake);
                intake.stopSpinner();
                intake.flipIn();
            }

            @Override
            public boolean isFinished() {
                return intake.atPosition();
            }
        };
    }

    public Command exhaustCommand(Intake intake) {
        return new CommandBase() {
            @Override
            public void initialize() {
                addRequirements(intake);
                intake.flipOut();
                intake.exhaust();
            }

            @Override
            public void end(boolean interrupted) {
                intake.stopSpinner();
            }

            @Override
            public boolean isFinished() {
                return intake.atPosition();
            }
        };
    }
    // TODO: Move to robot class and include retraction of extendo
    public Command transferCommand(Intake intake) {
        return retractCommand(intake)
                .andThen(new InstantCommand(intake::exhaust).withTimeout(1000))
                .andThen(new InstantCommand(intake::stopSpinner));
    }
}