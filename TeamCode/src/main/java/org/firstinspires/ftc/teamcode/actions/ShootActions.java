package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.commands.SorterCommands;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactManager;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class ShootActions {
    private ShootActions(){}

    private static boolean cancelRequested = false;
    private static boolean shooting = false;

    private static Command shootAll() {
        return new IfElseCommand(
                () -> !cancelRequested,
                new IfElseCommand(
                        ArtifactManager.INSTANCE::isEmpty,
                        finish(),
                        shootOne()
                                .then(new IfElseCommand(
                                        ArtifactManager.INSTANCE::isEmpty,
                                        finish(),
                                        turnSorter()
                                                .then(shootAll())))));
    }


    private static Command shootOne() {
        return realignSorter()
                .then(new WaitUntil(() ->
                        ShooterSubsystem.INSTANCE.atVel()
                                && SorterSubsystem.INSTANCE.atTarget()))
                .then(SorterCommands.kickerUp())
                .thenWait(0.15)
                .then(SorterCommands.removeLatestArtifact())
                .then(SorterCommands.kickerDown())
                .thenWait(0.15)
                .setInterruptible(false);
    }

    private static Command turnSorter(){
        return new IfElseCommand(
                ArtifactManager.INSTANCE::isMotifMode,
                SorterCommands.alignMotifArtifact(),
                SorterCommands.alignNextArtifact());
    }

    private static Command realignSorter() {
        return new IfElseCommand(
                ArtifactManager.INSTANCE::consumeMotifToggle,
                turnSorter());
    }

    private static Command finish() {
        return SorterCommands.home()
                .then(ShooterCommands.stop())
                .then(new LambdaCommand()
                        .setStart(() -> {
                            shooting = false;
                            cancelRequested = false;
                        })
                        .setIsDone(() -> true));
    }

    public static boolean isShooting() {
        return shooting;
    }

    public static void requestCancel(){
        cancelRequested = true;
    }

    public static Command startShooting() { //used to start shooting or restart after cancellation
        cancelRequested = false;
        shooting = true;
        return shootAll();
    }
}
