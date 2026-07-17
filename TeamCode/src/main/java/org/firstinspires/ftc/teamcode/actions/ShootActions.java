package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.commands.SorterCommands;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactManager;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.WaitUntil;

public class ShootActions {
    private ShootActions(){}

    public static Command shootOne() {
        return new WaitUntil(() ->
                ShooterSubsystem.INSTANCE.atVel() && SorterSubsystem.INSTANCE.atTarget())
                .then(SorterCommands.kickerUp())
                .thenWait(0.15)
                .then(SorterCommands.removeLatestArtifact())
                .then(SorterCommands.kickerDown())
                .thenWait(0.15)
                .setInterruptible(false);
    }
    public static Command shootAll() {
        return new IfElseCommand(
                ArtifactManager.INSTANCE::isEmpty,
                SorterCommands.home(),
                shootOne()
                        .then(SorterCommands.alignNextArtifact())
                        .then(ShootActions.shootAll()));
    }
    public static Command shootMotifArtifact(){
        return new WaitUntil(() ->
                ShooterSubsystem.INSTANCE.atVel())
                .then(SorterCommands.alignMotifArtifact())
                .then(SorterCommands.kickerUp())
                .thenWait(0.15)
                .then(SorterCommands.removeLatestArtifact())
                .then(SorterCommands.kickerDown())
                .thenWait(0.15)
                .setInterruptible(false);
    }
    public static Command shootMotif(){
        return new IfElseCommand(
                ArtifactManager.INSTANCE::isEmpty, SorterCommands.home(), shootMotifArtifact());
    }
    public static Command cancelShot(){
        return ShooterCommands.stop();
    }
}
