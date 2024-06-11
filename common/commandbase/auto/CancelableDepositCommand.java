package org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot.LiftProfiledCommand;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.subsystem.LiftSubsystem;

public class CancelableDepositCommand extends SequentialCommandGroup {
    private boolean cancelled = false;

    public CancelableDepositCommand(LiftSubsystem lift, SixConeAutoCommand command) {
        super(
                new LiftProfiledCommand(lift, LiftSubsystem.LiftState.HIGH),
                new WaitCommand(75),
                new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE),
                new WaitCommand(50),
                new InstantCommand(() -> command.canRetractDeposit = true),
                new WaitUntilCommand(() -> lift.getPos() > 510),
                new InstantCommand(() -> command.canRetractDeposit = false),
                new WaitUntilCommand(lift::isWithinTolerance),
                new WaitCommand(100),
                new InstantCommand(() -> lift.update(LiftSubsystem.LatchState.UNLATCHED)),
                new WaitCommand(75),
                new InstantCommand(() -> lift.update(LiftSubsystem.LiftState.RETRACTED))
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || cancelled;
    }

    public void cancel() {
        cancelled = true;
    }
}
