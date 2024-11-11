package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SimpleSubsystem;

@Config
public class Extendo extends SimpleSubsystem {

double MIN_POSITION = 0;
    public static final double MAX_POSITION = 1;

    private final Servo servo;


    private double position;

    public Extendo(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "extendo");
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
    }


    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("-------------\nExtendo");

    }

    public void extend() {
        position += 0.01;
    }
}