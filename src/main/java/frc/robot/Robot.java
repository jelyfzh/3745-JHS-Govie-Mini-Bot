package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.subsystems.Drivetrain;



public class Robot extends TimedRobot {
    
    //private RobotContainer m_robotContainer;
    //private static RobotContainer m_robotContainer = new RobotContainer();
    public final Drivetrain m_drivetrain = new Drivetrain();
    private final XboxController xboxController = new XboxController(0);
    private final edu.wpi.first.wpilibj.drive.MecanumDrive mecanumDrive = 
        new edu.wpi.first.wpilibj.drive.MecanumDrive(
            m_drivetrain.getFrontLeftMotor(),
            m_drivetrain.getRearLeftMotor(),
            m_drivetrain.getFrontRightMotor(),
            m_drivetrain.getRearRightMotor()
        );
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public Robot() {
        
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());

      // Configure autonomous sendable chooser
      m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

      SmartDashboard.putData("Auto Mode", m_chooser);
  
      configureButtonBindings();

      // Set the default command for the drivetrain to use the Xbox controller
      m_drivetrain.setDefaultCommand(
          new RunCommand(() -> m_drivetrain.driveCartesian(
              xboxController.getLeftY(), // Forward/backward (inverted Y-axis)
              xboxController.getLeftX(),  // Strafing (X-axis)
              xboxController.getRightX()  // Rotation (Z-axis)
          ), m_drivetrain)
      );
  }


  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  private void configureButtonBindings() {
        
    // Add button bindings here if needed

  }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public XboxController getXboxController() {
      return xboxController;
    }


  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
  

  public void setDefaultDriveCommand() {
    m_drivetrain.setDefaultCommand(
        new RunCommand(
            () -> {
              double ySpeed = -xboxController.getLeftY();
              double xSpeed = xboxController.getLeftX();
              double zRotation = xboxController.getRightX();
              driveCartesian(ySpeed, xSpeed, zRotation);
            },
            m_drivetrain));
  }

    


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

}
