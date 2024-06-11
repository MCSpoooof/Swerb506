package org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.subsystem.IntakeSubsystem;

public class PivotCommand extends InstantCommand {
    public PivotCommand(IntakeSubsystem intake, IntakeSubsystem.PivotState state) {
        super(
                () -> intake.update(state)
        );
    }
}
