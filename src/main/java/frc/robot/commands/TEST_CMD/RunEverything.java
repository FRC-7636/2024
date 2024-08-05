// package frc.robot.commands.TEST_CMD;

// import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;

// public class RunEverything extends Command {
//     private final Intake intake;
//     private final Shooter shooter;

//     public RunEverything(Intake intake, Shooter shooter) {
//         this.intake = intake;
//         this.shooter = shooter;

//         addRequirements(this.intake, this.shooter);
//     }

//     @Override
//     public void execute() {
//         shooter.shoot();
//         shooter.transport();
//         intake.startConvey();
//         intake.suck();
//     }
// }
