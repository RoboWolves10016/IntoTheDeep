package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;

@TeleOp(name = "Pedro Pathing TeleOp", group = "Test")
public class TeleOpDrive extends OpMode {

    private GamepadEx gamepad1;
    private GamepadEx gamepad2;
    private Follower follower;
    private Intake flipper;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private DashboardPoseTracker dashboardPoseTracker;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        gamepad1 = new GamepadEx(super.gamepad1);
        gamepad2 = new GamepadEx(super.gamepad2);

        follower = new Follower(hardwareMap);
        flipper = Intake.getInstance(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();

        configureBindings();
    }

    @Override
    public void loop() {

        telemetry.addData("Left Encoder Rotation: ", rightFront.getCurrentPosition());
        telemetry.addData("Right Encoder Rotation: ", rightRear.getCurrentPosition());
        telemetry.addData("Strafe Encoder Rotation: ", leftRear.getCurrentPosition());
        telemetry.update();
        follower.setTeleOpMovementVectors(-gamepad1.getLeftY(), -gamepad1.getLeftX(), -gamepad1.getRightX(), true);
        follower.update();

        if(gamepad2.getButton(GamepadKeys.Button.A)) {
            flipper.flipIn();
        }

        if(gamepad2.getButton(GamepadKeys.Button.B)) {
            flipper.flipIn();
        }
    }

    private void configureBindings() {
        gamepad1.getGamepadButton(GamepadKeys.Button.A)
                .while
    }
}
