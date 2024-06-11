package org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.subsystem.IntakeSubsystem;

public class FourbarProfiledCommand extends InstantCommand {
    public FourbarProfiledCommand(IntakeSubsystem intake, IntakeSubsystem.FourbarState state) {
        super(
                () -> intake.update(state)
        );
    }
}
