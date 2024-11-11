package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SimpleSubsystem;

@Config
public class Dumper extends SimpleSubsystem {

    private static final double DUMP_POSITION = 0.28;
    private static final double IDLE_POSITION = 0.49;

    private Servo servo;

    private double targetPosition = IDLE_POSITION;

    private boolean dumping = false;


    public Dumper(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "dumper");
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("-------------\nDumper");
        telemetry.addData("Dumping: ", dumping);
        telemetry.addData("Dumper Pos: ", servo.getPosition());
    }
    public void open() {
        targetPosition = DUMP_POSITION;
        dumping = true;
    }

    public void close() {
        targetPosition = IDLE_POSITION;
        dumping = false;
    }

    public void toggle() {
        if(dumping) {
            close();
        } else {
            open();
        }
    }

}
