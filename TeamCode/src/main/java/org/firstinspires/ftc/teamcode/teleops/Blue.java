package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.actions.IntakeActions;
import org.firstinspires.ftc.teamcode.actions.ShootActions;
import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.SorterCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactManager;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.components.Component;

public class Blue implements Component {
    private Follower follower;

    Button bButton = button(() -> gamepad1.b);

    Button dpadUpButton = button(() -> gamepad1.dpad_up);
    Button dpadDownButton = button(() -> gamepad1.dpad_down);
    Button dpadRightButton = button(() -> gamepad1.dpad_right);

    Button yButton = button(() -> gamepad2.y);
    Button xButton = button(() -> gamepad2.x);

    Button lbButton = button(() -> gamepad2.left_bumper);
    Button rbButton = button(() -> gamepad2.right_bumper);

    Button ltButton = button(() -> gamepad2.left_trigger_pressed);
    Button rtButton = button(() -> gamepad2.right_trigger_pressed);

    Button dpadRightButton2 = button(() -> gamepad2.dpad_right);
    Button dpadLeftButton = button(() -> gamepad2.dpad_left);

    private DoubleSupplier targetVelocity = () -> ShooterSubsystem.INSTANCE.getShortRpm();

    @Override
    public void preWaitForStart() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void postWaitForStart() {
        SorterCommands.home().schedule();
        follower.startTeleopDrive(true);
        VisionSubsystem.INSTANCE.setAlliance(true);
    }

    @Override
    public void postStartButtonPressed () {
        bButton.whenTrue(DriveCommands.faceTarget(() -> gamepad1.left_stick_x,
                        () -> -gamepad1.left_stick_y, 15, 128))
                        .whenFalse(DriveCommands.drive(() -> gamepad1.left_stick_x,
                                () -> -gamepad1.left_stick_y, () -> gamepad1.right_stick_x));

        yButton.toggleOnBecomesTrue()
                .whenBecomesTrue(
                        IntakeCommands.start()
                                .then(new IfElseCommand(
                                        () -> !IntakeActions.isCollecting(),
                                        new IfElseCommand(
                                                ShootActions::isShooting,
                                                IntakeActions.returnToIntake(targetVelocity),
                                                IntakeActions.startCollecting(targetVelocity)
                                        )
                                ))
                )
                .whenBecomesFalse(IntakeCommands.stop());

        xButton.whenBecomesTrue(IntakeCommands.reverse());

        lbButton.whenBecomesTrue(SorterCommands.manualOverride("G"));
        rbButton.whenBecomesTrue(SorterCommands.manualOverride("P"));

        ltButton.whenBecomesTrue(SorterCommands.adjustTargetTicks(50));
        rtButton.whenBecomesTrue(SorterCommands.adjustTargetTicks(-50));

        dpadUpButton.whenBecomesTrue(new IfElseCommand(ShootActions::isShooting, ShootActions.cancelShoot(),
            ShootActions.startShooting(() -> ShooterSubsystem.INSTANCE.getShortRpm())));
        dpadUpButton.whenBecomesTrue(() -> setTargetVelocity(() -> ShooterSubsystem.INSTANCE.getShortRpm()));

        dpadDownButton.whenBecomesTrue(new IfElseCommand(ShootActions::isShooting,
                ShootActions.cancelShoot(), ShootActions.startShooting(() ->
                ShooterSubsystem.INSTANCE.getLongRpm())));
        dpadDownButton.whenBecomesTrue(() -> setTargetVelocity(() -> ShooterSubsystem.INSTANCE.getLongRpm()));

        dpadRightButton.whenBecomesTrue(new IfElseCommand(ShootActions::isShooting,
                ShootActions.cancelShoot(), ShootActions.startShooting(() ->
                ShooterSubsystem.INSTANCE.getRPM(VisionSubsystem.INSTANCE.getDistance()))));
        dpadRightButton.whenBecomesTrue(() -> setTargetVelocity(() -> ShooterSubsystem.INSTANCE.getRPM(VisionSubsystem.INSTANCE.getDistance())));

        dpadLeftButton.whenBecomesTrue(ArtifactManager.INSTANCE::toggleMotifMode);
        dpadRightButton2.whenBecomesTrue(ShootActions.updateMotif());

        follower.update();
    }
    private void setTargetVelocity(DoubleSupplier targetVelocity){
        this.targetVelocity = targetVelocity;
    }
}
