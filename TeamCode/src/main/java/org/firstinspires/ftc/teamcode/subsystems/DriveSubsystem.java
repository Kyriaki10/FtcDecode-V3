package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.hardware.impl.MotorEx;

public class DriveSubsystem implements Subsystem {
    private static final Follower follower = Constants.createFollower(hardwareMap);
    private static final Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
    public static final DriveSubsystem INSTANCE = new DriveSubsystem(follower, limelight);


    private final MotorEx leftFront = new MotorEx("leftFront")
            .brakeMode();
    private final MotorEx leftBack = new MotorEx("leftBack")
            .brakeMode();
    private final MotorEx rightFront = new MotorEx("rightFront")
            .brakeMode()
            .reversed();

    private final MotorEx rightBack = new MotorEx("rightBack")
            .brakeMode()
            .reversed();

    private final PIDController headingPID = new PIDController(HEADING_COEFFICIENTS);
    private final ElapsedTime pidTimer = new ElapsedTime();
    public static final PIDCoefficients HEADING_COEFFICIENTS = new PIDCoefficients(2, 0, 0.21);
    private double lastKnownHeading = Double.NaN;
    private final ElapsedTime refreshTimer = new ElapsedTime();

    public DriveSubsystem(Follower follower, Limelight3A limelight) {
    }

    public void drive(double x, double y, double rotation){
        double lf = y + x + rotation;
        double lb = y - x + rotation;
        double rf = y - x - rotation;
        double rb = y + x - rotation;

        double max = Math.max(Math.abs(lf), Math.max(Math.abs(lb), Math.max(Math.abs(rf), Math.abs(rb))));
        if (max > 1.0) {
            lf /= max;
            lb /= max;
            rf /= max;
            rb /= max;
        }
        setPowers(lf, lb, rf, rb);
    }
    private void setPowers(double lf, double lb, double rf, double rb){
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }
    public void stop(){
        setPowers(0,0,0,0);
    }

    public double outputTargetTurn(double x, double y){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            lastKnownHeading = follower.getPose().getHeading() + Math.toRadians(tx);
            refreshTimer.reset();

            return clamp(headingPID.update(Math.toRadians(tx), getDt()));
        }
        if (!Double.isNaN(lastKnownHeading) && refreshTimer.milliseconds() < 300) {
            double currentHeading = follower.getPose().getHeading();
            Angle headingError = Angle.fromRad(lastKnownHeading - currentHeading);
            headingError = headingError.normalized();

            return clamp(headingPID.update(headingError.inRad, getDt()));
        }
        return outputFacePoint(x, y);
    }

    public double outputFacePoint(double x, double y){
        Pose currentPose = follower.getPose();

        double dx = x - currentPose.getX();
        double dy = y - currentPose.getY();

        double targetHeading = Math.atan2(dy, dx);
        double currentHeading = currentPose.getHeading();

        Angle headingError = Angle.fromRad(targetHeading - currentHeading);
        headingError = headingError.normalized();

        return clamp(headingPID.update(headingError.inRad, getDt()));
    }
    private double getDt() {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt > 0.5) {
            headingPID.reset();
            return 0.01;
        }

        return dt;
    }
    private double clamp(double output) {
        return Math.max(-1.0, Math.min(1.0, output));
    }
}
