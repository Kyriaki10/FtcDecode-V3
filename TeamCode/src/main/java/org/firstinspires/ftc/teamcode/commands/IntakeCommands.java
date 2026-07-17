package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public final class IntakeCommands {
    private IntakeCommands(){}
    private static final double REVERSE_DURATION = 0.5;

    public static Command start(){
        return new LambdaCommand("Start Intake")
                .setUpdate(IntakeSubsystem.INSTANCE::start)
                .requires(IntakeSubsystem.INSTANCE);
    }

    public static Command reverse(){
        ElapsedTime timer = new ElapsedTime();

        return new LambdaCommand("Reverse Intake")
                .setStart(timer::reset)
                .setUpdate(IntakeSubsystem.INSTANCE::reverse)
                .setIsDone(() -> timer.seconds() > REVERSE_DURATION)
                .setStop(Interrupted -> IntakeSubsystem.INSTANCE.stop())
                .requires(IntakeSubsystem.INSTANCE);
    }
    public static Command stop() {
        return new LambdaCommand("Stop Intake")
                .setStart(IntakeSubsystem.INSTANCE::stop)
                .setIsDone(() -> true)
                .requires(IntakeSubsystem.INSTANCE);
    }

}
