package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.DriveSystem;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class HighRow extends SequentialCommandGroup {

    public HighRow(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem, Boolean armOut) {
        addCommands(
            //places game piece, skips docking, then passes mobility bonus line

            //game piece
            new SequentialCommandGroup(
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem),
                new WaitCommand(0.5),
                new ArmRotate(16000, armRotateSystem)
            ),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(2),
                        new ArmExtend(-750000, armExtendSystem)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(2),
                        new ArmRotate(15000, armRotateSystem)
                    ),
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
                new RepeatCommand(new ArmRotate(31000, armRotateSystem))
            ),
            new ParallelDeadlineGroup(
            new ArmRotate(0, armRotateSystem),
            new DriveDistance(-6, driveSystem))
            // new ParallelDeadlineGroup(new WaitCommand(4), new DriveDistance(5, driveSystem))
        );
    }
}