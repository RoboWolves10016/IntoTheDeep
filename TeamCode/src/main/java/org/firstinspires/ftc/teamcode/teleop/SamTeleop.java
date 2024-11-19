package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Dumper;
import org.firstinspires.ftc.teamcode.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.List;

@Disabled
@TeleOp(name="Sam TeleOp")
public class SamTeleop extends OpMode {

    private double driveSpeed;

    List<LynxModule> allHubs;
    private Follower follower;
    private Intake intake;
    private Extendo extendo;
    private Lift lift;
    private Claw claw;
    private Dumper dumper;

    private PIDController headingController;

    private boolean preIntake = false;

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = new Follower(hardwareMap);
        intake = Robot.getIntake(hardwareMap);
//        extendo = Robot.getExtendo(hardwareMap);
//        lift = Robot.getLift(hardwareMap);
//        claw = Robot.getClaw(hardwareMap);
//        dumper = Robot.getDumper(hardwareMap);


        CommandScheduler.getInstance().registerSubsystem(intake);


        headingController = new PIDController(0.01, 0, 0);
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        driverControls();
        operatorControls();


        CommandScheduler.getInstance().run();

        intake.updateTelemetry(telemetry);
        telemetry.update();
    }

    private void driverControls() {
        if (gamepad1.right_trigger > 0.1) {
            driveSpeed = 0.25;
        } else {
            driveSpeed = 1.0;
        }

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, -gamepad1.right_stick_x * driveSpeed / 2, false);
    }

    private void operatorControls() {
        if (gamepad2.right_trigger > 0.1) {
            intake.intake();
            preIntake = true;
        } else if (gamepad2.right_bumper || preIntake) {
            intake.preIntake();
            preIntake = true;
        } else if (gamepad2.left_bumper) {
            intake.transferPosition();
            preIntake = false;
        } else if (gamepad2.left_trigger > 0.1) {
            intake.idlePosition();
            preIntake = false;
        }

        intake.setExhaust(gamepad2.b);
    }

    private void checkIntakeCollision() {
        if (intake.getState().equals(Intake.IntakeState.TRANSFER)) {
            intake.idlePosition();
        }
    }

}
