package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.commands.SorterCommands;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactManager;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class ShootActions {
    private ShootActions(){}

    private static boolean cancelRequested = false;
    private static boolean shooting = false;
    private static final ElapsedTime timeOutTimer = new ElapsedTime();

    private static Command shootAll(DoubleSupplier targetVelocity) {
        return new IfElseCommand(
                () -> !cancelRequested,
                new IfElseCommand(
                        ArtifactManager.INSTANCE::isEmpty,
                        finish(),
                        shootOne(targetVelocity)
                                .then(new IfElseCommand(
                                        ArtifactManager.INSTANCE::isEmpty,
                                        finish(),
                                        turnSorter()
                                                .then(shootAll(targetVelocity))))));
    }


    private static Command shootOne(DoubleSupplier targetVelocity) {
        return realignSorter()
                .then(new WaitUntil(() ->
                        SorterSubsystem.INSTANCE.atTarget()))
                .and(ShooterCommands.updateTargetVelocity(targetVelocity))
                .until(() ->
                        ShooterSubsystem.INSTANCE.atVel()
                                && SorterSubsystem.INSTANCE.atTarget())
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
        shooting = false;
    }

    public static Command startShooting(DoubleSupplier targetVelocity) {
        cancelRequested = false;
        shooting = true;
        return shootAll(targetVelocity);
    }
    public static Command cancelShoot() {
        return new LambdaCommand()
                .setStart(ShootActions::requestCancel)
                .setIsDone(() -> true);
    }
    public static Command updateMotif(){
        timeOutTimer.reset();
        return new LambdaCommand("Detect Current Motif")
                .setStart(VisionSubsystem.INSTANCE::start)
                .setUpdate(() -> {
                    String[] motif = VisionSubsystem.INSTANCE.detectMotif();
                    if (motif != null) {
                        ArtifactManager.INSTANCE.updateMotif(motif);
                    }
                })
                .setIsDone(() ->
                        VisionSubsystem.INSTANCE.isMotifDetected()
                                || timeOutTimer.seconds() > 0.5);
    }
}
