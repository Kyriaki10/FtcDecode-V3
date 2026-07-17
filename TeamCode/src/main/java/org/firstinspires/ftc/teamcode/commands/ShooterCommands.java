package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public final class ShooterCommands {
    private ShooterCommands() {}

    public static Command setTargetVelocity(double rpm){
        return new LambdaCommand("Set Velocity")
                .setStart(() -> ShooterSubsystem.INSTANCE.setVelocity(rpm))
                .setIsDone(() -> true)
                .requires(ShooterSubsystem.INSTANCE);
    }
    public static Command stop(){
        return new LambdaCommand("Stop Shooter")
                .setStart(ShooterSubsystem.INSTANCE::stop)
                .setIsDone(() -> true)
                .requires(ShooterSubsystem.INSTANCE);
    }
    public static Command spinUp(double rpm) {
        return new LambdaCommand("Spin Up Shooter")
                .setStart(() -> ShooterSubsystem.INSTANCE.setVelocity(rpm))
                .setIsDone(ShooterSubsystem.INSTANCE::atVel)
                .requires(ShooterSubsystem.INSTANCE);
    }
}
