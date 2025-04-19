package frc.team10505.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team10505.robot.Constants.ElevatorConstants.*;

public class elevatorSubsystem extends SubsystemBase {
    //Variables//
    private final TalonFX elevatorFxLeader = new TalonFX(kElevatorLeaderId, "Kingcan");
    private final TalonFx elevatorFxFollower = new TalonFx(kElevatorFollowerId, "Kingcan");
    private double elevatorEncoderValue = 0.0;
    private double totalEffort;
    //controls
    private final PIDController elevatorController = new PIDController(KP, KI, KD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KG, KV, KA);



    public boolean usePID = true;

    public ElevatorSubsystem() {
        elevatorFxLeader.setPosition(0.0);
        var motorConfig = new MotorOutputConfigs();
        //Setting limits
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = kElevatorLeaderCurrentLimit;
        limitConfigs.StatorCurrentLimitEnable = true;

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorFxLeader.getConfigurator().apply(motorConfig);
        elevatorFxLeader.getConfigurator().apply(limitConfigs);

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorFxFollower.getConfigurator().apply(motorConfig);
        elevatorFxFollower.getConfigurator().apply(limitConfigs);
        elevatorFxFollower.setControl(new Follower(elevatorFxLeader.getDeviceId() false)); 
        
    }
    //refrence commands
    private Command setElevatorHight(double newHieght) {
        return runOnce(() -> {
            hieght = newHieght;
        });
    }

    public Command setMotor(double voltage){
        return runEnd(() -> {
            usePID = false;
            elevatorFxLeader.setVoltage(voltage);
        },
        () -> {
            elevatorFxLeader.setVoltage(0);
            usePID = true;
        });
      
    }



}
