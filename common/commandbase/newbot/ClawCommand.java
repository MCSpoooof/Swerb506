package org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.subsystem.IntakeSubsystem;

public class ClawCommand extends InstantCommand {
    public ClawCommand(IntakeSubsystem intake, IntakeSubsystem.ClawState state) {
        super(
                () -> intake.update(state)
        );
    }
}
