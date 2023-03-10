package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.DriveSystem;

public class Basic extends SequentialCommandGroup {

    public Basic(DriveSystem driveSystem) {
        addCommands(
                new DriveDistance(-2, driveSystem));
    }
}