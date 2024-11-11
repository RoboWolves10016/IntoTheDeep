package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SimpleSubsystem extends SubsystemBase {

    public abstract void init();
    public abstract void updateTelemetry(Telemetry telemetry);
}
