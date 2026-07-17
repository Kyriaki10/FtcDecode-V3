package org.firstinspires.ftc.teamcode.commands;

import android.content.ContentQueryMap;

import org.firstinspires.ftc.teamcode.subsystems.ArtifactManager;
import org.firstinspires.ftc.teamcode.subsystems.SorterSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public final class SorterCommands {
    private SorterCommands() {}

    public static Command home(){
        return new LambdaCommand("Home Sorter")
                .setStart(SorterSubsystem.INSTANCE::home)
                .setIsDone(() -> SorterSubsystem.INSTANCE.atTarget())
                .requires(SorterSubsystem.INSTANCE);
    }
    public static Command advanceToNextSlot(){
        return new LambdaCommand("Move To Next Empty Slot")
                .setStart(SorterSubsystem.INSTANCE::advanceSlot)
                .setIsDone(() -> SorterSubsystem.INSTANCE.atTarget())
                .requires(SorterSubsystem.INSTANCE);
    }
    public static Command alignNextArtifact(){
        return new LambdaCommand("Align With The Next Artifact")
                .setStart(() -> SorterSubsystem.INSTANCE
                        .rotateToSlot(ArtifactManager.INSTANCE.getNextArtifactSlot()))
                .setIsDone(() -> SorterSubsystem.INSTANCE.atTarget())
                .requires(SorterSubsystem.INSTANCE, ArtifactManager.INSTANCE);
    }
    public static Command alignMotifArtifact(){
        return new LambdaCommand("Align With The Motif Artifact")
                .setStart(() -> SorterSubsystem.INSTANCE
                        .rotateToSlot(ArtifactManager.INSTANCE.getMotifSlot()))
                .setIsDone(() -> SorterSubsystem.INSTANCE.atTarget())
                .requires(SorterSubsystem.INSTANCE, ArtifactManager.INSTANCE);
    }
    public static Command adjustTargetTicks(int delta){
        return new LambdaCommand("Adjust Sorter")
                .setStart(() -> SorterSubsystem.INSTANCE.adjustTargetTicks(delta))
                .setIsDone(() -> true)
                .requires(SorterSubsystem.INSTANCE);
    }
    public static Command removeLatestArtifact(){
        return new LambdaCommand("Remove Latest Artifact")
                .setStart(() -> {
                    SorterSubsystem.INSTANCE.home();
                    ArtifactManager.INSTANCE.removeArtifact
                            (SorterSubsystem.INSTANCE.getCurrentSlot());
                })
                .setIsDone(() -> SorterSubsystem.INSTANCE.atTarget())
                .requires(SorterSubsystem.INSTANCE, ArtifactManager.INSTANCE);
    }
    public static Command kickerUp(){
        return new LambdaCommand("Push Artifact")
                .setStart(SorterSubsystem.INSTANCE::kickerUp)
                .setIsDone(() -> true)
                .requires(SorterSubsystem.INSTANCE);
    }
    public static Command kickerDown(){
        return new LambdaCommand("Transfer Returns to Down Position")
                .setStart(SorterSubsystem.INSTANCE::kickerDown)
                .setIsDone(() -> true)
                .requires(SorterSubsystem.INSTANCE);
    }
}
