package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.Convertor;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class SorterSubsystem implements Subsystem {
    public static SorterSubsystem INSTANCE;

    private final RobotHardware hw;
    private final MotorEx sorter = new MotorEx("sorter").brakeMode();
    private final ServoEx sorterTransfer = new ServoEx("sorter");
    private final PIDController sorterPID;
    final SorterRecovery recovery = new SorterRecovery();

    // ===== PID TUNING =====
    private static final double kP = 0.0005; //0.0003 0.0005
    private static final double kI = 0;
    private static final double kD = 0.000019; //0.000004 0.000018

    // ====== PID PARAMS =========
    private double error;
    private double dt;

    // ===== CONTROL PARAMS =====
    public static final double MIN_POWER = 0.1;   // overcome static friction
    private static final int SlOW_ZONE = 700;        // ticks
    private static final int TARGET_TOLERANCE = 200;  // ticks
    private static final long SETTLE_TIME = 20;      // ms

    // ===== STUCK DETECTION PARAMS =====
    public double targetTicks;
    private long lastTime;
    private long stableSince = 0;

    // ===== SLOT HANDLING =======

    private final static double STARTING_OFFSET = -50;
    public final static double[] initialOffsets =
            {STARTING_OFFSET, STARTING_OFFSET + 120, STARTING_OFFSET + 240};
    private int currentSlot = 0;
    private boolean offsetApplied = true;

    public SorterSubsystem(RobotHardware hw) {
        INSTANCE = this;
        this.hw = hw;

        sorter.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sorterPID = new PIDController(new PIDCoefficients(kP, kI, kD));

        targetTicks = (int) sorter.getCurrentPosition();
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void periodic(){
        updateMeasurements();
        updateStuckDetection();
        updatePid ();
    }

    private void updatePid() {
        switch (recovery.getState()){
            case NORMAL: runPID();
            break;
            case REVERSING: sorter.setPower(-0.3);
            break;
            case RECOVERING: sorter.setPower(0);
            break;
        }
    }
    private void updateMeasurements() {
        error = targetTicks - sorter.getCurrentPosition();
        long now = System.currentTimeMillis();
        dt = Math.max((now - lastTime) / 1000.0, 0.001);
        lastTime = now;
    }

    private void runPID() {
        double voltage = hw.getBatteryVoltage();
        double dynamicKD = 0.0000165 - (voltage - 12.0) * 0.000000125;
        sorterPID.setCoefficients(new PIDCoefficients(kP, kI, dynamicKD));

        double power = sorterPID.update(error, dt);

        double scale = Math.min(1.0, Math.abs(error) / SlOW_ZONE);
        power *= scale;

        // Minimum power clamp (prevents stalling & wiggle)
        if (Math.abs(power) < MIN_POWER && Math.abs(error) > TARGET_TOLERANCE) {
            power = Math.signum(power) * MIN_POWER;
        }
        power = Math.max(-1.0, Math.min(1.0, power));

        sorter.setPower(power);
    }

    private void updateStuckDetection() {
        recovery.update(error);
        if (recovery.justEntered(SorterRecovery.State.RECOVERING)) {
            targetTicks -= Convertor.toTicks(SorterRecovery.REVERSE_DISTANCE);
            sorterPID.reset();
        }
        if (recovery.justEntered(SorterRecovery.State.NORMAL)) {
            targetTicks += Convertor.toTicks(SorterRecovery.REVERSE_DISTANCE);
            sorterPID.reset();
        }
    }

    public void moveDegrees(double degrees) {
        targetTicks += Convertor.toTicks(degrees);
        sorterPID.reset();
        stableSince = 0;
    }
    public void rotateToSlot(int slot) {
        if (slot < 0 || slot > 2) return;
        if (!offsetApplied) {
            moveDegrees(initialOffsets[slot]);
            offsetApplied = true;
            currentSlot = slot;
            return;
        }

        int delta = (slot - currentSlot + 3) % 3;
        int step = delta == 1 ? 1 : delta == 2 ? -1 : 0;
        if (step != 0) moveDegrees(step * RobotHardware.DEG_PER_SLOT);
        currentSlot = slot;
    }
    public void advanceSlot(){
        moveDegrees(120);
        currentSlot = (currentSlot + 1) % 3;
    }
    public void home(){
        if (offsetApplied) {
            moveDegrees(-STARTING_OFFSET);
            offsetApplied = false;
            currentSlot = 0;
        }
    }

    public void adjustTargetTicks(int delta) {
        targetTicks += delta;
    }

    public void resetPID() {
        sorterPID.reset();
        sorter.setPower(0);
        targetTicks = (int) sorter.getCurrentPosition();
        stableSince = 0;
        lastTime = System.currentTimeMillis();
    }

    public boolean atTarget() {
        if (Math.abs(error) < 190) {
            if (stableSince == 0) {
                stableSince = System.currentTimeMillis();
            }
            return (System.currentTimeMillis() - stableSince) > SETTLE_TIME;
        } else {
            stableSince = 0;
            return false;
        }
    }
    public int getCurrentSlot(){
        return currentSlot;
    }
    public void kickerUp(){
        sorterTransfer.setPosition(RobotHardware.transferPush);
    }
    public void kickerDown(){
        sorterTransfer.setPosition(RobotHardware.transferIdle);
    }
}

