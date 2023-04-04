package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.ClawSystem;
import frc.robot.commands.auto.GyroTurnAngle;
import frc.robot.subsystems.DriveSystem;

public class Basic extends SequentialCommandGroup {

    public Basic(DriveSystem driveSystem, ClawSystem clawSystem) {
        addCommands(
            new GyroTurnAngle(90, driveSystem)
        );
    }
}