package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.subsystems.Subsystem;

public class VisionSubsystem implements Subsystem {
    public static final VisionSubsystem INSTANCE = new VisionSubsystem();
    private VisionSubsystem() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    private final Limelight3A limelight;
    private static LLResult result;

    private static final double TARGET_HEIGHT = 74; // in cm
    private static final double CAMERA_HEIGHT = 38.5;
    private static final double CAMERA_ANGLE = 12.0; // deg

    private static final int RED_PIPELINE = 8;
    private static final int BLUE_PIPELINE = 9;
    private static final int OBELISK_PIPELINE = 1;
    private static int alliancePipeline;
    private static int currentPipeline;

    private static boolean update;

    private int loopCount = 0;
    private static boolean motifDetected;

    @Override
    public void periodic(){
        if (!update) return;
        loopCount++;

        if ((loopCount & 1) == 0) {
            result = limelight.getLatestResult();
        }
    }

    public void setAlliance(boolean blue){
        alliancePipeline = blue ? BLUE_PIPELINE : RED_PIPELINE;
    }

    public boolean hasResult(){
        return result != null && result.isValid();
    }

    public double getHeadingError(){
        setPipeline(alliancePipeline);
        if(hasResult()){
            return result.getTx();
        }
        return -1;
    }
    public double getVerticalError(){
        setPipeline(alliancePipeline);
        if(hasResult()){
            return result.getTy();
        }
        return -1;
    }

    public int getMotifId(){
        setPipeline(OBELISK_PIPELINE);
        if(!hasResult()) return -1;

        for(LLResultTypes.FiducialResult f : result.getFiducialResults()){
            return f.getFiducialId();
        }
        return -1;
    }

    public String[] detectMotif() {
        switch (getMotifId()) {
            case 21:
                motifDetected = true;
                return new String[]{"G", "P", "P"};
            case 22:
                motifDetected = true;
                return new String[]{"P", "G", "P"};
            case 23:
                motifDetected = true;
                return new String[]{"P", "P", "G"};
            default:
                motifDetected = false;
                return null;
        }
    }
    private void setPipeline(int pipeline) {
        if (currentPipeline != pipeline) {
            limelight.pipelineSwitch(pipeline);
            currentPipeline = pipeline;
            result = null;
        }
    }
    public void stop(){
        update = false;
        limelight.stop();
    }
    public void start(){
        result = null;
        update = true;
        limelight.start();
    }
    public boolean isMotifDetected(){
        return motifDetected;
    }

    public double getDistance(){
        double ty = getVerticalError();

        if(ty == -1){
            return -1;
        }
        double angle = Math.toRadians(CAMERA_ANGLE + ty);

        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angle);
    }
}
