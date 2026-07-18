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
    private boolean firstShot = false;

    private static final double SHORT_RPM = 1950;
    private static final double LONG_RPM = 2340;

    private static final double[] DISTANCE_TABLE = {75, 165, 200, 330};
    private static final double[] RPM_TABLE = {1900, 2000, 2060, 2600};

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
        if (firstShot){
            targetTPS *= 0.97;
            firstShot = false;
        }
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

    public double getShortRpm(){return SHORT_RPM;}
    public double getLongRpm(){return LONG_RPM;}

    public void prepareForFirstShot(){
        firstShot = true;
    }

    public double getRPM(double distance) {
        if (distance <= DISTANCE_TABLE[0]) {
            return RPM_TABLE[0];
        }
        if (distance >= DISTANCE_TABLE[DISTANCE_TABLE.length - 1]) {
            return RPM_TABLE[RPM_TABLE.length - 1];
        }

        for (int i = 0; i < DISTANCE_TABLE.length - 1; i++) {
            if (distance >= DISTANCE_TABLE[i] && distance <= DISTANCE_TABLE[i + 1]) {
                double t = (distance - DISTANCE_TABLE[i]) / (DISTANCE_TABLE[i + 1] - DISTANCE_TABLE[i]);
                return RPM_TABLE[i] + t * (RPM_TABLE[i + 1] - RPM_TABLE[i]);
            }
        }

        return RPM_TABLE[RPM_TABLE.length - 1];
    }
}
