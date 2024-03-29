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
import frc.robot.commands.auto.TurnDistance;
import frc.robot.subsystems.DriveSystem;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.commands.auto.ArmRotateHold;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class PreloadDockHigh extends SequentialCommandGroup {

    public PreloadDockHigh(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
        addCommands(
            //places game piece, skips docking, then passes mobility bonus line

            //game piece
            new SequentialCommandGroup(
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem),
                new WaitCommand(0.25),
                new ArmRotate(25000, armRotateSystem),
                new ParallelRaceGroup(
                    new WaitCommand(2.5),
                    new ArmExtend(-600000, armExtendSystem),
                    new RepeatCommand(new ArmRotate(26000, armRotateSystem))
                )
            ),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    //new WaitCommand(0.25),
                    new DriveDistance(0.75, driveSystem),
                    new InstantCommand(() -> clawSystem.solenoid1.set(Value.kForward), clawSystem),
                    new ParallelDeadlineGroup(
                        new WaitCommand(1.5),
                        new ArmExtend(-50000, armExtendSystem)
                    )
                ),
                new RepeatCommand(new ArmRotateHold(30000, armRotateSystem))
                ),
    
           //Movement and docking (guessed values)
           new SequentialCommandGroup(
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(
                    new ArmRotate(3000, armRotateSystem),
                    new DriveDistance(-0.5, driveSystem)),
            new TurnDistance(0.125, -0.125, driveSystem),
            new DriveDistance(5, driveSystem),
            new ParallelDeadlineGroup(
                    new WaitCommand(4),
                    new DockingCommand(driveSystem)
            )
            
        )
        );
    }
}