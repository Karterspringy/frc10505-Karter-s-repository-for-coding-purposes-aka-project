package frc.team10505.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.ElevatorSubsystem;

public class robotContainer {
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    //operater interface
    private final CommandXboxController xboxController1; //=// new CommandXboxController(0);
    private final CommandXboxController xboxController2; //= new CommandXboxController(1);
    private final CommandJoystick joystick1; //= //new CommandJoystick(0);
    private final CommandJoystick joystick2; //= new CommandJoystick(1);
    //supersticture
    
public robotContainer() {
    if (Utils.isSimulation()) {
        xboxController1 = new CommandXboxController(0);
        xboxController2 = new CommandXboxController(1);
        joystick1 = new CommandJoystick(0);
        joystick2 = new CommandJoystick(1);
    }else{
        xboxController1 = new CommandXboxController(0);
        xboxController2 = new CommandXboxController(1);
        joystick1 = new CommandJoystick(0);
        joystick2 = new CommandJoystick(1);
    }
   
}
    

    public void configButtonBindings() {
        if (Utils.isSimulation()) {
            joystick1.button(1).onTrue(elevatorSubsystem.setElevatorHight(0));
            joystick1.button(2).onTrue(elevatorSubsystem.setElevatorHight(1));
            joystick1.button(3).onTrue(elevatorSubsystem.setElevatorHight(2));
            joystick1.button(4).onTrue(elevatorSubsystem.setElevatorHight(3));
        }else{
            xboxController2.a().onTrue(elevatorSubsystem.setElevatorHight(8.0));
            xboxController2.b().onTrue(elevatorSubsystem.setElevatorHight(8.0));
            xboxController2.x().onTrue(elevatorSubsystem.setElevatorHight(8.0));
            xboxController2.y().onTrue(elevatorSubsystem.setElevatorHight(8.0));
        }
    }



}
