package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake extends SimpleSubsystem {

    private static final double TRANSFER_LOWER_POS = 0;
    private static final double TRANSFER_UPPER_POS = 0;
    private static final double PRE_INTAKE_LOWER_POS = 0;
    private static final double PRE_INTAKE_UPPER_POS = 0;
    private static final double INTAKE_LOWER_POS = 0;
    private static final double INTAKE_UPPER_POS = 0;

    private static final double INTAKE_SPEED = 1;
    private static final double EXHAUST_SPEED = 1;

    // This enum represents a state that the intake is commanded to be in
    // The possible states are Transfer, Pre-Intake, and Intake
    // We used an enum constructor so that we could link positions to each state
    // These positions are accessible because the variables are public
    public enum IntakeState {
        TRANSFER(TRANSFER_LOWER_POS, TRANSFER_UPPER_POS, 0),
        PRE_INTAKE(PRE_INTAKE_LOWER_POS, PRE_INTAKE_UPPER_POS, 0),
        INTAKE(INTAKE_LOWER_POS, INTAKE_UPPER_POS, INTAKE_SPEED);

        public final double lowerTarget;
        public final double upperTarget;
        public final double speed;

        // Here's the aforementioned enum constructor
        private IntakeState(double lowerTarget, double upperTarget, double intakeSpeed) {
            this.lowerTarget = lowerTarget;
            this.upperTarget = upperTarget;
            this.speed = intakeSpeed;
        }
    }

    private final Servo lowerJoint;
    private final Servo upperJoint;
    private final CRServo intake;

    private IntakeState currentState = IntakeState.TRANSFER;

    private boolean exhaust = false;

    public Intake(HardwareMap hwMap) {
        lowerJoint = hwMap.servo.get("axonmax");
        upperJoint = hwMap.servo.get("axonmini");
        intake = hwMap.get(CRServo.class, "spinner");
    }

    @Override
    public void periodic() {
        lowerJoint.setPosition(currentState.lowerTarget);
        upperJoint.setPosition(currentState.upperTarget);
        intake.setPower(currentState.speed);
        if(exhaust) {
            intake.setPower(EXHAUST_SPEED);
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake State: ", currentState.name());
    }

    public void intake() {
        currentState = IntakeState.INTAKE;
    }

    public void preIntake() {
        currentState = IntakeState.PRE_INTAKE;
    }

    public void transferPosition() {
        currentState = IntakeState.TRANSFER;
    }

}