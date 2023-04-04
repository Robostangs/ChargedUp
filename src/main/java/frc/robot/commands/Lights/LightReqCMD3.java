// package frc.robot.commands.Lights;


// import com.ctre.phoenixpro.hardware.TalonFX;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Lighting;

// public class LightReqCMD extends CommandBase {
//     private final Lighting mLighting = Lighting.getInstance();
//     Boolean Cone;
//     Boolean Cube;
//     String reqPiece;
    
//     double PWMVal; // Arbitrary switch to 0.99 later
//     Timer timer;
//     SequentialCommandGroup loop;
//     double blinkSpeed = 0.5;
//     int angle;

    
//     public LightReqCMD(int angle) {
//         this.angle = angle; 
//         // Lighting.lastLight = PWMVal;
//         addRequirements(mLighting);

//         timer = new Timer();

//         loop = new SequentialCommandGroup(
//             new InstantCommand(() -> mLighting.setLights(PWMVal)).andThen(
//                 new WaitCommand(blinkSpeed).andThen(
//                     new InstantCommand(() -> mLighting.setLights(mLighting.killLights)).andThen(
//                         new WaitCommand(blinkSpeed)
//                     )
//                 )
//             )
//         );
            
//     }
    
//     @Override
//     public void initialize() {
//         if (angle == 270) {
//             PWMVal = mLighting.coneLight;
//             this.setName("Requesting Piece: Cone");
//         } if (angle == 90) {
//             PWMVal = mLighting.cubeLight;
//             this.setName("Requesting Piece: Cube");
//         }
//         mLighting.lastLight = PWMVal;
//         timer.restart();
//     }

//     @Override
//     public void execute() {
//         if (timer.get() <= 10) {
//             System.out.println("Timer: " + timer.get());
//             if (!loop.isFinished()) {
//                 loop.schedule();
//             }
//         } else {
//             // new InstantCommand(() -> mLighting.setLights(PWMVal));
//             System.out.println("Lights stale");
//             this.cancel();
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         System.out.println("Lights set");
//         // mLighting.setLights(mLighting.lastLight);
//         // System.out.println(mLighting.lastLight);
//     }
// }