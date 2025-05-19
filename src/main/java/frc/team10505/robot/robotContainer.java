package frc.team10505.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;

public class robotContainer {
    private final ElevatorSubsystem elevatorSubsystem; //= new ElevatorSubsystem();
    private final AlgaeSubsystem algaeSubsystem;
    //operater interface
    private final CommandXboxController xboxController1; //=// new CommandXboxController(0);
    private final CommandXboxController xboxController2; //= new CommandXboxController(1);
    private final CommandJoystick joystick1; //= //new CommandJoystick(0);
    private final CommandJoystick joystick2; //= new CommandJoystick(1);
    
    private SendableChooser<Command> autoChoose;

    
public robotContainer() {
    if (Utils.isSimulation()) {
        xboxController1 = new CommandXboxController(0);
        xboxController2 = new CommandXboxController(1);
        joystick1 = new CommandJoystick(0);
        joystick2 = new CommandJoystick(1);
        elevatorSubsystem = new ElevatorSubsystem();
        algaeSubsystem = new AlgaeSubsystem();
    }else{
        xboxController1 = new CommandXboxController(0);
        xboxController2 = new CommandXboxController(1);
        joystick1 = new CommandJoystick(0);
        joystick2 = new CommandJoystick(1);
        elevatorSubsystem = new ElevatorSubsystem();
        algaeSubsystem = new AlgaeSubsystem();
    }
   configButtonBindings();
   autoChoose = AutoBuilder.buildAutoChooser();
   SmartDashboard.putData("Auton",autoChoose);
   
}
    

    public void configButtonBindings() {
        if (Utils.isSimulation()) {
            //elevator
            joystick1.button(1).onTrue(elevatorSubsystem.setElevatorHight(0));
            joystick1.button(2).onTrue(elevatorSubsystem.setElevatorHight(1));
            joystick1.button(3).onTrue(elevatorSubsystem.setElevatorHight(2));
            joystick1.button(4).onTrue(elevatorSubsystem.setElevatorHight(3));
            //pivot
            joystick2.button(1).onTrue(algaeSubsystem.setAngle(-90));
            joystick2.button(2).onTrue(algaeSubsystem.setAngle(-45));
            joystick2.button(3).onTrue(algaeSubsystem.setAngle(0));
            joystick2.button(4).onTrue(algaeSubsystem.setAngle(90));
        }else{
            //elevator
            xboxController2.a().onTrue(elevatorSubsystem.setElevatorHight(0));
            xboxController2.b().onTrue(elevatorSubsystem.setElevatorHight(0));
            xboxController2.x().onTrue(elevatorSubsystem.setElevatorHight(0));
            xboxController2.y().onTrue(elevatorSubsystem.setElevatorHight(0));
            //pivot
            xboxController1.a().onTrue(algaeSubsystem.setAngle(0));
            xboxController1.b().onTrue(algaeSubsystem.setAngle(0));
            xboxController1.x().onTrue(algaeSubsystem.setAngle(0));
            xboxController1.y().onTrue(algaeSubsystem.setAngle(0));
        }
    }




}
