package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.DockingCommand;

import frc.robot.commands.auto.DriveDistance;

import frc.robot.subsystems.DriveSystem;


public class DockOnly extends SequentialCommandGroup {

    public DockOnly(DriveSystem driveSystem) {
        //Starts facing charging station, moves forward and docks
        addCommands(
            new SequentialCommandGroup(
                new DriveDistance(5, driveSystem),
                new ParallelDeadlineGroup(new WaitCommand(8), new DockingCommand(driveSystem))
            )
            
        );
    }
}