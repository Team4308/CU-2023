package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.TurnDistance;
import frc.robot.subsystems.DriveSystem;

import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class PreloadDock extends SequentialCommandGroup {

    public PreloadDock(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
        addCommands(
            //Places game piece, turns around and docks

            //game piece (copied from PreloadMob)
            new SequentialCommandGroup(
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem),
                new WaitCommand(0.5),
                new ArmRotate(29000, armRotateSystem)
            ),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(4),
                        new ArmExtend(-280000, armExtendSystem)
                    ),
                    //Group structure copied from PreloadMob, have questions
                    new ParallelCommandGroup(       
                        new InstantCommand(() -> clawSystem.solenoid1.set(Value.kForward), clawSystem)
                    ),
                    new SequentialCommandGroup(
                        new WaitCommand(1),
                        new ParallelDeadlineGroup(
                            new WaitCommand(4),
                            new ArmExtend(0, armExtendSystem)
                        )
                    )
                ),
                new RepeatCommand(new ArmRotate(29000, armRotateSystem))
            ),

            //Movement and docking (guessed values)
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new TurnDistance(5, -5, driveSystem),
                new DriveDistance(5, driveSystem),
                new ParallelDeadlineGroup(
                        new WaitCommand(4),
                        new DockingCommand(driveSystem)
                )
                
            )
        );
    }
}