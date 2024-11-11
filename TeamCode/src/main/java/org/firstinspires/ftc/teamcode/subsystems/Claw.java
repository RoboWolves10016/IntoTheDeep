package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SimpleSubsystem;

@Config
public class Claw extends SimpleSubsystem {

    private static final double OPEN_POSITION = 0.28;
    private static final double CLOSED_POSITION = 0.49;

    private Servo servo;

    private double targetPosition = CLOSED_POSITION;

    private boolean open = false;


    public Claw(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "claw");
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("-------------\nClaw");
        telemetry.addData("Claw Open: ", open);
        telemetry.addData("Claw Pos: ", servo.getPosition());
    }
    public void open() {
        targetPosition = OPEN_POSITION;
        open = true;
    }

    public void close() {
        targetPosition = CLOSED_POSITION;
        open = false;
    }

    public void toggle() {
        if(open) {
            close();
        } else {
            open();
        }
    }

}
