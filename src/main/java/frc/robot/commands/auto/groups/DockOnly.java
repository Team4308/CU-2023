package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.DriveSystem;

import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class DockOnly extends SequentialCommandGroup {

    public DockOnly(DriveSystem driveSystem) {
        addCommands(
            new ParallelDeadlineGroup(new WaitCommand(4), new DriveDistance(5, driveSystem))
        );
    }
}