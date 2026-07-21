package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
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

public class IntakeActions {
    private IntakeActions() {}
    private static boolean collecting = true;

    private static Command intakeSequence(DoubleSupplier targetVelocity) {
         return collectOne()
                 .repeatedly()
                 .until(() -> ArtifactManager.INSTANCE.isFull() || ShootActions.isShooting())
                .then(prepareForShot(targetVelocity));
    }
    private static Command collectOne() {
        return new WaitUntil(() ->
                ArtifactManager.INSTANCE.detectArtifact(SorterSubsystem.INSTANCE.getCurrentSlot()))
                .thenWait(0.05)
                .then(new IfElseCommand(
                        () -> !ArtifactManager.INSTANCE.isFull(),
                        SorterCommands.advanceToNextSlot()));
    }

    private static Command prepareForShot(DoubleSupplier targetVelocity){
        return new LambdaCommand()
                .setStart(() -> {
                    ShooterSubsystem.INSTANCE.prepareForFirstShot();
                    VisionSubsystem.INSTANCE.start();
                    collecting = false;
                })
                .setIsDone(() -> true)
                .then(new WaitUntil(ArtifactManager.INSTANCE::isFull)
                        .then(IntakeCommands.reverse())
                        .then(IntakeCommands.stop())
                        .then(new IfElseCommand(
                                ArtifactManager.INSTANCE::isMotifMode,
                                SorterCommands.alignMotifArtifact(),
                                SorterCommands.alignNextArtifact())
                                .and(ShooterCommands.setTargetVelocity(targetVelocity))));
    }

    public static boolean isCollecting(){
        return collecting;
    }

    public static Command startCollecting(DoubleSupplier targetVelocity){
        return new LambdaCommand()
                .setStart(() -> {
                    collecting = true;
                    VisionSubsystem.INSTANCE.stop();
                })
                .setIsDone(() -> true)
                .then(intakeSequence(targetVelocity));
    }

    public static Command returnToIntake(DoubleSupplier targetVelocity){
        return ShooterCommands.stop()
                .and(SorterCommands.removeLatestArtifact())
                .then(startCollecting(targetVelocity));
    }
}
