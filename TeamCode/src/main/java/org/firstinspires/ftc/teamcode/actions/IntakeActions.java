package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.commands.SorterCommands;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactManager;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.WaitUntil;

public class IntakeActions {
    private IntakeActions() {}

    public static Command intakeSequence(DoubleSupplier targetVelocity) {
         return collectOne()
                 .repeatedly()
                 .until(ArtifactManager.INSTANCE::isFull)
                .then(prepareForShot(targetVelocity));
    }
    private static Command collectOne() {
        return new WaitUntil(() ->
                ArtifactManager.INSTANCE.detectArtifact(SorterSubsystem.INSTANCE.getCurrentSlot()))
                .thenWait(0.05)
                .then(new IfElseCommand(() -> !ArtifactManager.INSTANCE.isFull(),
                        SorterCommands.advanceToNextSlot()));
    }

    private static Command prepareForShot(DoubleSupplier targetVelocity){
        ShooterSubsystem.INSTANCE.prepareForFirstShot();
        return new WaitUntil(ArtifactManager.INSTANCE::isFull)
                .then(IntakeCommands.reverse())
                .then(IntakeCommands.stop())
                .then(SorterCommands.alignNextArtifact())
                .and(ShooterCommands.setTargetVelocity(targetVelocity.getAsDouble()));
    }
    public static Command returnToIntake(){
        return SorterCommands.removeLatestArtifact()
                .and(ShooterCommands.stop());
    }
}
