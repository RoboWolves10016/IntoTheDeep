package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Intake Test", group="test")
public class IntakeTest extends OpMode {
    private Servo axonMax;
    private Servo axonMini;
    private CRServo spinner;

    private double miniTarget = 0.5;
    private double maxTarget = 0.5;
    private double spinnerSpeed = 0;

    @Override
    public void init() {
        axonMax = hardwareMap.get(Servo.class, "axonmax");
        axonMini = hardwareMap.get(Servo.class, "axonmini");
        spinner = hardwareMap.get(CRServo.class, "spinner");
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) {
            axonMax.setPosition(1);
        }

        if(gamepad1.dpad_down) {
            axonMax.setPosition(0);
        }

        if(gamepad1.dpad_right) {
            axonMini.setPosition(1);
        }

        if(gamepad1.dpad_left) {
            axonMini.setPosition(0);
        }

        if(gamepad1.start) {
            axonMini.setPosition(0.5);
            axonMax.setPosition(0.5);
        }

        if(gamepad1.right_trigger > 0.1) {
            spinnerSpeed = 1;
        } else if(gamepad1.left_trigger > 0.1) {
            spinnerSpeed = -1;
        } else {
            spinnerSpeed = 0;
        }

        axonMax.setPosition(maxTarget);
        axonMax.setPosition(miniTarget);
        spinner.setPower(spinnerSpeed);

        telemetry.addData("Max Target: ", maxTarget);
        telemetry.addData("Mini Target: ", miniTarget);
        telemetry.addData("Spinner Target: ", spinnerSpeed);
    }
}
