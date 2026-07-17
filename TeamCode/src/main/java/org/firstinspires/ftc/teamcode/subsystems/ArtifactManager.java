package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Objects;

public class ArtifactManager {
    public static final ArtifactManager INSTANCE = new ArtifactManager();
    ArtifactManager() {}
    private final ColorRangeSensor colorSensor = hardwareMap.get(ColorRangeSensor.class, " color sensor");
    private final String[] storedArtifacts = new String[3];
    private int artifactCount = 0;
    private String[] motif = {"G", "P", "P"};
    private int motifNextIndex = 0;
    private final ElapsedTime detectTimer = new ElapsedTime();
    private static final long SLOT_DEBOUNCE_MS = 300;
    private boolean isSampling = false;
    private long sumG = 0, sumB = 0, sumR = 0;
    private int sampleCount = 0;
    private static final long SAMPLING_DURATION_MS = 100;
    private final ElapsedTime samplingTimer = new ElapsedTime();
    private boolean lastDetected = false;
    private boolean motifMode = false;
    private boolean motifToggleTriggered = false;

    public void manualDetect(int slot, String color) {
        if (artifactCount >= storedArtifacts.length) return;
        if (detectTimer.milliseconds() < SLOT_DEBOUNCE_MS) return;

        if (isSlotEmpty(slot)) {
            storeArtifact(slot, color);
            detectTimer.reset();
        }
    }
    public boolean detectArtifact(int slot) {
        double distance = colorSensor.getDistance(DistanceUnit.MM);
        boolean detectedNow = distance < 75 && distance > 20;
        boolean risingEdge = detectedNow && !lastDetected;
        boolean canDetect = isSlotEmpty(slot);

        if (risingEdge && !isSampling && canDetect && detectTimer.milliseconds() > 400) {
            isSampling = true;
            samplingTimer.reset();
            sumG = 0; sumB = 0; sumR = 0;
            sampleCount = 0;
        }
        if (isSampling) {
            sumG += colorSensor.green();
            sumB += colorSensor.blue();
            sumR += colorSensor.red();
            sampleCount++;

            if (samplingTimer.milliseconds() > SAMPLING_DURATION_MS) {
                isSampling = false;
                detectTimer.reset();

                if (artifactCount < storedArtifacts.length && sampleCount > 0) {
                    double avgG = (double) sumG / sampleCount;
                    double avgB = (double) sumB / sampleCount;
                    double avgR = (double) sumR / sampleCount;

                    String color = (avgG > avgR * 1.2 && avgG > avgB) ? "G" : "P";

                    if (isSlotEmpty(slot)) {
                        storeArtifact(slot, color);
                        return true;
                    }
                }
            }
        }

        lastDetected = detectedNow;
        return false;
    }

    public void toggleMotifMode() {
        motifMode = !motifMode;
        motifToggleTriggered = true;
    }
    public boolean isMotifMode(){
        return motifMode;
    }
    public boolean consumeMotifToggle() {
        if (!motifToggleTriggered) return false;

        motifToggleTriggered = false;
        return true;
    }

    private void storeArtifact(int currentSlot, String color) {
        if (artifactCount >= storedArtifacts.length) return;

        if (currentSlot == -1) {
            syncArtifactCount();
            return;
        }

        if (isSlotEmpty(currentSlot)) storedArtifacts[currentSlot] = color;
        syncArtifactCount();
    }

    public int getNextArtifactSlot() {
        if (this.artifactCount < 1) return -1;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != null) {
                return i;
            }
        }
        return -1;
    }

    public int getMotifSlot(){
        if (this.artifactCount < 1) return -1;
        String targetArtifact = motif[motifNextIndex % motif.length];
        for (int i = 0; i < this.storedArtifacts.length; i++) {
            if (Objects.equals(this.storedArtifacts[i], targetArtifact)) {
                return i;
            }
        }
        return  getNextArtifactSlot();
    }

    public void removeArtifact(int currentSlot) {
        if (currentSlot < 0 || currentSlot >= storedArtifacts.length) return;

        if (storedArtifacts[currentSlot] != null) {
            storedArtifacts[currentSlot] = null;
            motifNextIndex = (motifNextIndex + 1) % motif.length;

            syncArtifactCount();
        }
    }

    public void seedArtifacts(String... colors) {
        resetArtifacts();
        int count = Math.min(colors.length, storedArtifacts.length);
        for (int i = 0; i < count; i++) {
            if (colors[i] == null) {
                continue;
            }
            storeArtifact(i, colors[i]);
        }
    }
    public boolean isSlotEmpty(int slot) {
        return storedArtifacts[slot] == null;
    }
    public boolean isEmpty() {
        return artifactCount == 0;
    }

    public boolean isFull() {
        return artifactCount == storedArtifacts.length;
    }
    public void setMotif(String[] targetMotif){
        motif = Arrays.copyOf(targetMotif, 3);
    }

    private void syncArtifactCount() {
        int count = 0;
        for (String artifact : storedArtifacts) {
            if (artifact != null) {
                count++;
            }
        }
        artifactCount = count;
    }

    public int getArtifactCount(){
        return artifactCount;
    }
    public boolean isLastArtifact(){
        return artifactCount == 2;
    }

    public void resetArtifacts() {
        isSampling = false;
        sampleCount = 0;
        lastDetected = false;
        detectTimer.reset();
        samplingTimer.reset();
        artifactCount = 0;
        motifNextIndex = 0;
        Arrays.fill(storedArtifacts, null);
    }
}
