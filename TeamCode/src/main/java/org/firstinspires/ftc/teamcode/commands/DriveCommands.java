package org.firstinspires.ftc.teamcode.commands;


import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public final class DriveCommands {
    private DriveCommands() {}
    public static Command drive (DoubleSupplier x,
                                 DoubleSupplier y,
                                 DoubleSupplier rotation) {
        return new LambdaCommand("Drive")
                .setUpdate(() ->
                        DriveSubsystem.INSTANCE.drive
                                (x.getAsDouble(),
                                        y.getAsDouble(),
                                        rotation.getAsDouble()))
                .requires(DriveSubsystem.INSTANCE);
    }
    public static Command faceTarget (DoubleSupplier x,
                                   DoubleSupplier y,
                                   double goalX,
                                   double goalY) {
        return new LambdaCommand("Face Target")
                .setUpdate(() -> DriveSubsystem.INSTANCE.drive(x.getAsDouble(), y.getAsDouble(),
                        DriveSubsystem.INSTANCE.outputTargetTurn(goalX, goalY)))
                .requires(DriveSubsystem.INSTANCE);
    }
    public static Command stop () {
        return new LambdaCommand("Stop")
                .setStart(DriveSubsystem.INSTANCE::stop)
                .setIsDone(() -> true)
                .requires(DriveSubsystem.INSTANCE);
    }
}
