package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.Convertor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterSubsystem implements Subsystem {
    private final RobotHardware hw;

    public static ShooterSubsystem INSTANCE;
    public boolean firstShot = false;
    public ShooterSubsystem(RobotHardware hw) {
        this.hw = hw;
        INSTANCE = this;
    }

    private final MotorEx leftShooter = new MotorEx("leftShooter")
            .reversed()
            .floatMode();
    private final MotorEx rightShooter = new MotorEx("rightShooter")
            .floatMode();
    private final MotorGroup shooter = new MotorGroup(leftShooter, rightShooter);
    public static double kP = 0.02;
    public static double kI = 0;
    public static double kD = 0.02;
    public static double kV = 10.1;

    public double targetRPM;
    public double targetTPS;
    private static final double NOMINAL_BATTERY = 13.0;
    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(kP, kI, kD)
            .build();
    @Override
    public void periodic(){
        double voltage = hw.getBatteryVoltage();

        double kvDynamic = kV * (NOMINAL_BATTERY / voltage);

        double feedforward = kvDynamic * targetTPS;

        double pid = shooterController.calculate(shooter.getState());

        shooter.setPower(feedforward + pid);
    }
    public void setVelocity(double rpm){
        targetRPM = rpm;
        targetTPS = Convertor.shooterRPMtoTPS(targetRPM);

        shooterController.setGoal(new KineticState(0.0, targetTPS));
    }
    public boolean atVel(){
        return Math.abs(shooter.getVelocity() - targetTPS)
                <= Convertor.shooterRPMtoTPS(200);
    }
    public void stop() {
        setVelocity(0);
        shooterController.reset();
    }
    public void prepareForFirstShot(){
        firstShot = true;
    }
}
