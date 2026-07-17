package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Convertor;

public class SorterRecovery {

    // -------------------------
    // Recovery Constants
    // -------------------------

    private static final double STUCK_ERROR_THRESHOLD = 5.0;
    private static final double STUCK_DETECTION_TIME = 2.0;

    private static final double TARGET_TOLERANCE = 10.0;

    private static final double REVERSE_TIME = 0.25;
    static final double REVERSE_DISTANCE = 10.0;
    private static final double RECOVERY_TIME = 0.20;

    // -------------------------
    // Runtime Variables
    // -------------------------

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime errorTimer = new ElapsedTime();

    private double lastError;

    public enum State {
        NORMAL,
        REVERSING,
        RECOVERING
    }

    private State state = State.NORMAL;
    private State previousState = State.NORMAL;


    public void update(double error) {
        // Reset timer whenever the error changes significantly.
        if (Math.abs(error - lastError) > STUCK_ERROR_THRESHOLD) {
            lastError = error;
            errorTimer.reset();
        }

        previousState = state;

        switch (state) {
            case NORMAL:
                if (errorTimer.seconds() > STUCK_DETECTION_TIME
                        && Math.abs(error) > TARGET_TOLERANCE) {
                    setState(State.REVERSING);
                    // Reverse intake here.
                }
                break;
            case REVERSING:
                if (stateTimer.seconds() > REVERSE_TIME) {
                    setState(State.RECOVERING);
                    // targetOffsetTicks -= Convertor.toTicks(REVERSE_DISTANCE);
                    // Reset PID here.
                }
                break;
            case RECOVERING:
                if (stateTimer.seconds() > RECOVERY_TIME) {
                    setState(State.NORMAL);
                    // targetOffsetTicks += Convertor.toTicks(REVERSE_DISTANCE);
                    // Reset PID here.
                }
                break;
        }
    }
    public State getState(){
        return state;
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }
    public boolean justEntered(State state) {
        return this.state == state && previousState != state;
    }
}