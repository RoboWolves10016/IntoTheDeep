package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Intake Test3", group="test")
public class IntakeTest3 extends OpMode {
    private Servo axonMax;
    private Servo axonMini;
    private Servo slideServo;
    private CRServo spinner;

    private double miniTarget = 0.45;
    private double maxTarget = 0.70;
    private double slideTarget = 0.5;
    private double spinnerSpeed = 0;
    private boolean gotoPre = false, inIntake = false;

    @Override
    public void init() {
        axonMax = hardwareMap.get(Servo.class, "axonmax");
        axonMini = hardwareMap.get(Servo.class, "axonmini");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        slideServo = hardwareMap.get(Servo.class, "slideservo");
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_up) {
            maxTarget = maxTarget + 0.001;
        }
        if(gamepad1.dpad_down) {
            maxTarget = maxTarget - 0.001;
        }
        if(gamepad1.dpad_left) {
            miniTarget = miniTarget - 0.001;
        }
        if(gamepad1.dpad_right) {
            miniTarget = miniTarget + 0.001;
        }
        if(gamepad1.y) {
           slideTarget = slideTarget + 0.001;
        }
        if(gamepad1.a) {
            slideTarget = slideTarget - 0.001;
        }
        if (gamepad2.right_stick_y > 0.02 && slideTarget <= 0.8) {
            slideTarget = slideTarget + 0.002;
        }
        if (gamepad2.right_stick_y < -0.02 && slideTarget >= 0.176) {
            slideTarget = slideTarget - 0.002;
        }


        axonMax.setPosition(maxTarget);
        axonMax.setPosition(maxTarget);
        axonMini.setPosition(miniTarget);
        spinner.setPower(spinnerSpeed);
        slideServo.setPosition(slideTarget);

        telemetry.addData("Max Target: ", maxTarget);
        telemetry.addData("Mini Target: ", miniTarget);
        telemetry.addData("Spinner Target: ", spinnerSpeed);
        telemetry.addData("Slide Target: ", slideTarget);
        telemetry.addData("DPad Up/Down",  "%b / %b",gamepad1.dpad_up, gamepad1.dpad_down);
        telemetry.addData("DPad Right/Left",  "%b / %b",gamepad1.dpad_right, gamepad1.dpad_left);
        telemetry.addData("Device Name: ", axonMini.getController().getDeviceName());


    }
}
