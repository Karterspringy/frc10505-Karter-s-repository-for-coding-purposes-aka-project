package frc.team10505.robot.subsystems;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    //Variables//
    public static final int kElevatorFollowerId = 52;
    public static final int kElevatorLeaderId = 53;
    public static final int kElevatorLeaderCurrentLimit = 40;
    //PID
    public static double KP = 0.32;
    public static double KI = 0.0;
    public static double KD = 0.0;
   //Suff in voltage feed forward
   public static double KS = 0.0;
   public static double KG = 0.0;
   public static double KV = 0.0;
   public static double KA = 0.0;
   
    private final TalonFX elevatorFxLeader;// = new TalonFX(kElevatorLeaderId, "Kingcan");
    private final TalonFX elevatorFxFollower;// = new TalonFX(kElevatorFollowerId, "Kingcan");
    //Encoders both real and simulated.
    private DutyCycleEncoder elevatorEncoderValue = new DutyCycleEncoder(1);
    private double totalEffort;
    //controls
    private final PIDController elevatorController = new PIDController(KP, KI, KD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(KG, KV, KA);
    //Operator interface
    private final SendableChooser<Double> elevatorHeight = new SendableChooser<>();
    private double Height = 0.0;
    //Sim vars
    private final Mechanism2d elevMech = new Mechanism2d(1.5, 5.0);
    private MechanismRoot2d elevRoot = elevMech.getRoot("elevRoot", 0.25, 0.0);
    private MechanismLigament2d elevatorViz = elevRoot.append(new MechanismLigament2d("elevator", 0.75, 90));
    private ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2), 12, 10, Units.inchesToMeters(1.5), 0, 30, true, Height, null);

  
        
    
      




    public boolean usePID = true;

    public ElevatorSubsystem() {
        if (Utils.isSimulation()) {
            elevatorFxLeader = new TalonFX(kElevatorLeaderId);
            elevatorFxFollower = new TalonFX(kElevatorFollowerId);
        }else{
            elevatorFxLeader = new TalonFX(kElevatorLeaderId, "kingCan");
            elevatorFxFollower = new TalonFX(kElevatorFollowerId, "Kingcan");
        }
        
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
        elevatorFxFollower.setControl(new Follower(elevatorFxLeader.getDeviceID(), false));

        
        
    }
    //refrence commands
    public Command setElevatorHight(double newHeight) {
        return runOnce(() -> {
            Height = newHeight;
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

  

    public double calculatevoltage(){
        if(Utils.isSimulation()){
            return elevatorFeedforward.calculate(0)+ elevatorController.calculate(Height);
        }else{
            return elevatorFeedforward.calculate(elevatorEncoderValue.get(),0) + elevatorController.calculate(elevatorEncoderValue.get(), Height);
        }
        }

    @Override
    public void periodic(){

        elevatorFxLeader.setVoltage(calculatevoltage());
        

        if(Utils.isSimulation()){
            elevatorSim.setInput(elevatorFxLeader.getMotorVoltage().getValueAsDouble());
            elevatorSim.update(0.01); 
            elevatorViz.setLength(elevatorSim.getPositionMeters());
         }

         SmartDashboard.putNumber("height", Height);
         SmartDashboard.putNumber("heightEncoder", elevatorEncoderValue.get());

    }

}
