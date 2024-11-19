package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SimpleSubsystem;

@Config
public class Intake extends SimpleSubsystem {

    private static final double IDLE_MAX_POS = 0.930;
    private static final double IDLE_MINI_POS = 0.355;

    private static final double TRANSFER_MAX_POS = 1.0;
    private static final double TRANSFER_MINI_POS = 0.635;

    private static final double PRE_INTAKE_MAX_POS = 0.415;
    private static final double PRE_INTAKE_MINI_POS = 0.155;

    private static final double INTAKE_MAX_POS = 0.25;
    private static final double INTAKE_MINI_POS = 0.14;

    public static final double IDLE_SPEED = -0.1;
    private static final double INTAKE_SPEED = -1;
    private static final double EXHAUST_SPEED = 1;


    public enum IntakeState {
        IDLE(IDLE_MAX_POS, IDLE_MINI_POS, IDLE_SPEED),
        TRANSFER(TRANSFER_MAX_POS, TRANSFER_MINI_POS, IDLE_SPEED),
        PRE_INTAKE(PRE_INTAKE_MAX_POS, PRE_INTAKE_MINI_POS, IDLE_SPEED),
        INTAKE(INTAKE_MAX_POS, INTAKE_MINI_POS, INTAKE_SPEED);

        public final double lowerTarget;
        public final double upperTarget;
        public final double speed;

        IntakeState(double lowerTarget, double upperTarget, double intakeSpeed) {
            this.lowerTarget = lowerTarget;
            this.upperTarget = upperTarget;
            this.speed = intakeSpeed;
        }
    }

    private final Servo axonMax;
    private final Servo axonMini;
    private final CRServo intake;

    private IntakeState currentState = IntakeState.IDLE;

    private boolean exhaust = false;

    public Intake(HardwareMap hwMap) {
        axonMax = hwMap.servo.get("axonmax");
        axonMini = hwMap.servo.get("axonmini");
        intake = hwMap.get(CRServo.class, "spinner");
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        axonMax.setPosition(currentState.lowerTarget);
        axonMini.setPosition(currentState.upperTarget);
        intake.setPower(currentState.speed);
        if(exhaust) {
            intake.setPower(EXHAUST_SPEED);
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("-------------\nIntake");
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

    public void idlePosition() {
        currentState = IntakeState.IDLE;
    }
    public void setExhaust(boolean exhaust) {
        this.exhaust = exhaust;
    }

    public IntakeState getState() {
        return currentState;
    }

}