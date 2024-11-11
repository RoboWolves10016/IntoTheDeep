package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.subsystems.Extendo;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp(name = "Pedro Pathing TeleOp", group = "Test")
public class TeleOpDrive extends OpMode {

    private Follower follower;

//    private Intake intake;
//    private Extendo extendo;
//    private Lift lift;


    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
//        follower = new Follower(hardwareMap);
//        intake = new Intake(hardwareMap);
//        extendo = new Extendo(hardwareMap);
//        lift = new Lift(hardwareMap);
//
//        CommandScheduler.getInstance().registerSubsystem(intake, extendo, lift);

//        AnalogInput
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

     //   telemetry.addData("Left Encoder Rotation: ", rightFront.getCurrentPosition());
    //    telemetry.addData("Right Encoder Rotation: ", rightRear.getCurrentPosition());
    //    telemetry.addData("Strafe Encoder Rotation: ", leftRear.getCurrentPosition());

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
//        CommandScheduler.getInstance().run();

        telemetry.update();
    }

    private void configureBindings() {

    }
}
