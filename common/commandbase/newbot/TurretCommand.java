package org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.subsystem.IntakeSubsystem;

public class TurretCommand extends InstantCommand {
    public TurretCommand(IntakeSubsystem intake, IntakeSubsystem.TurretState state) {
        super(
                () -> intake.update(state)
        );
    }
}
