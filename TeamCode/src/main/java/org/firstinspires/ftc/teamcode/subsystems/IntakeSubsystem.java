package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class IntakeSubsystem implements Subsystem {
    public static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
    private final ElapsedTime recoveryTimer = new ElapsedTime();
    private final SorterRecovery recovery = SorterSubsystem.INSTANCE.recovery;
    private static final double REVERSE_DURATION = 1.0;
    IntakeSubsystem() {}
    private final MotorEx intake = new MotorEx("intake")
            .brakeMode()
            .reversed();

    @Override
    public void periodic(){
        if (recovery.justEntered(SorterRecovery.State.REVERSING))
            recoveryTimer.reset();

        if(recoveryTimer.seconds() < REVERSE_DURATION)
            reverseRecovery();
    }

    public void start(){
        intake.setPower(1);
    }
    public void reverse(){
        intake.setPower(-1);
    }
    public void reverseRecovery(){
        intake.setPower(-0.5);
    }
    public void stop(){
        intake.setPower(0);
    }
}
