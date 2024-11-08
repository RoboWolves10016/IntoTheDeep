package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Extendo;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;

@TeleOp(name = "Pedro Pathing TeleOp", group = "Test")
public class TeleOpDrive extends OpMode {

    private Follower follower;
    private Intake intake;
    private Extendo extendo;
    private Lift lift;


    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        intake = new Intake(hardwareMap);
        extendo = new Extendo(hardwareMap);
        lift = new Lift(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(intake, extendo, lift);

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

     //   telemetry.addData("Left Encoder Rotation: ", rightFront.getCurrentPosition());
    //    telemetry.addData("Right Encoder Rotation: ", rightRear.getCurrentPosition());
    //    telemetry.addData("Strafe Encoder Rotation: ", leftRear.getCurrentPosition());

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.update();
    }
}
