package frc.robot.commands.auton.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.auton.DriveDistance;
import frc.robot.subsystems.DriveSystem;


public class Balance extends SequentialCommandGroup {

    public Balance(DriveSystem driveSystem) {
        addCommands(
            // TODO: Make command for balancing
            new ParallelDeadlineGroup(new WaitCommand(2), new DriveDistance(-0.5, driveSystem))
            
        );
    }
}