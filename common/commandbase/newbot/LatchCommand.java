package org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.subsystem.LiftSubsystem;

public class LatchCommand extends InstantCommand {
    public LatchCommand(LiftSubsystem lift, LiftSubsystem.LatchState state) {
        super(
                () -> lift.update(state)
        );
    }
}
