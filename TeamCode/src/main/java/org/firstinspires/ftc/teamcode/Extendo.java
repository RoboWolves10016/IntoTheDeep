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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Extendo extends SimpleSubsystem {

    public static final double MIN_POSITION = 0;
    public static final double MAX_POSITION = 1;

    private final Servo servo;


    private double position;

    public Extendo(HardwareMap hwMap) {
        servo = hwMap.servo.get("extendo");
    }


    @Override
    public void periodic() {
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Extendo Pos Deg: ", currentPosition);
        telemetry.addData("Extendo Pos Deg: ", targetPosition);
        telemetry.update();
    }

    public void extend() {
        position += 0.1;
    }
}