package org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot.teleop_commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot.ClawCommand;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot.FourbarProfiledCommand;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot.LatchCommand;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot.PivotCommand;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.newbot.TurretCommand;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Swerb506.common.commandbase.subsystem.LiftSubsystem;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(IntakeSubsystem intake, LiftSubsystem lift) {
        super(
                new LatchCommand(lift, LiftSubsystem.LatchState.UNLATCHED),
                new TurretCommand(intake, IntakeSubsystem.TurretState.INWARDS),
                new PivotCommand(intake, IntakeSubsystem.PivotState.PRE_TRANSFER),
                new FourbarProfiledCommand(intake, IntakeSubsystem.FourbarState.PRE_TRANSFER),
                new InstantCommand(() -> intake.setTargetPosition(-5)),
                new WaitUntilCommand(() -> intake.isWithinTolerance() && Math.abs(intake.fourbarMotionState.v) <= 0.5),
                new FourbarProfiledCommand(intake, IntakeSubsystem.FourbarState.TRANSFER),
                new PivotCommand(intake, IntakeSubsystem.PivotState.TRANSFER),
                new WaitUntilCommand(() -> intake.fourbarMotionState.v == 0),
                new ClawCommand(intake, IntakeSubsystem.ClawState.CLEAR),
                new FourbarProfiledCommand(intake, IntakeSubsystem.FourbarState.CLEAR),
                new WaitCommand(250),
                new LatchCommand(lift, LiftSubsystem.LatchState.INTERMEDIATE)
        );
    }
}
