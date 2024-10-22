package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Claw extends SimpleSubsystem {
    private static final double OPEN_POSITION = 0.5;
    private static final double CLOSED_POSITION = 0.5;
    private Servo servo;

    private double clawPosition = 0;
    private double targetPosition = CLOSED_POSITION;

    private static Claw instance;
    public static Claw getInstance(HardwareMap hwMap) {
        if(instance == null) {
            instance = new Claw(hwMap);
        }
        return instance;
    }

    private Claw(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "claw");
        servo.getPosition();
    }

    @Override
    public void periodic() {
        clawPosition = servo.getPosition();
        servo.setPosition(targetPosition);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Claw Pos: ", servo.getPosition());
        telemetry.update();
    }

    public Command closeClawCommand(Claw claw) {
        return new CommandBase() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public void initialize() {
                addRequirements(claw);
                timer.reset();
                targetPosition = CLOSED_POSITION;
            }

            @Override
            public boolean isFinished() {
                return timer.milliseconds() > 500;
            }
        };
    }

    public Command openClawCommand(Claw claw) {
        return new CommandBase() {
            ElapsedTime timer = new ElapsedTime();
            @Override
            public void initialize() {
                addRequirements(claw);
                timer.reset();
                targetPosition = OPEN_POSITION;
            }

            @Override
            public boolean isFinished() {
                return timer.milliseconds() > 500;
            }
        };
    }
}
