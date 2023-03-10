package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.DriveSystem;

public class Balance extends SequentialCommandGroup {

    public Balance(DriveSystem driveSystem) {
        addCommands(
                new ParallelDeadlineGroup(new WaitCommand(2), new DriveDistance(-0.5, driveSystem)),
                new DockingCommand(driveSystem));
    }
}