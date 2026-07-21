package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public final class ShooterCommands {
    private ShooterCommands() {}

    public static Command setTargetVelocity(DoubleSupplier rpm){
        return new LambdaCommand("Set Velocity")
                .setStart(() -> ShooterSubsystem.INSTANCE.setVelocity(rpm.getAsDouble()))
                .setIsDone(() -> false)
                .requires(ShooterSubsystem.INSTANCE);
    }
    public static Command stop(){
        return new LambdaCommand("Stop Shooter")
                .setStart(ShooterSubsystem.INSTANCE::stop)
                .setIsDone(() -> true)
                .requires(ShooterSubsystem.INSTANCE);
    }
    public static Command updateTargetVelocity(DoubleSupplier rpm) {
        return new LambdaCommand("Follow Shooter Velocity")
                .setUpdate(() ->
                        ShooterSubsystem.INSTANCE.setVelocity(rpm.getAsDouble())
                )
                .setIsDone(() -> false)
                .requires(ShooterSubsystem.INSTANCE);
    }
    public static Command spinUp(DoubleSupplier rpm) {
        return new LambdaCommand("Spin Up Shooter")
                .setStart(() -> ShooterSubsystem.INSTANCE.setVelocity(rpm.getAsDouble()))
                .setIsDone(ShooterSubsystem.INSTANCE::atVel)
                .requires(ShooterSubsystem.INSTANCE);
    }
}
