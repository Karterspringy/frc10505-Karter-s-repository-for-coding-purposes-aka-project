package frc.team10505.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    // Variables//
    public static final int kElevatorFollowerId = 11;
    public static final int kElevatorLeaderId = 10;
    public static final int kElevatorLeaderCurrentLimit = 40;

    private final TalonFX elevatorFxLeader;// = new TalonFX(kElevatorLeaderId, "Kingcan");
    private final TalonFX elevatorFxFollower;// = new TalonFX(kElevatorFollowerId, "Kingcan");
    // Encoders both real and simulated.
    private DutyCycleEncoder elevatorEncoderValue = new DutyCycleEncoder(1);
    //private double totalEffort;
    private double simEncoder = 0.0;
    // controls
    private final PIDController elevatorController; //= new PIDController(0, 0, 0);
    private final ElevatorFeedforward elevatorFeedforward; // = new ElevatorFeedforward(0, 0, 0);
    // Operator interface
    //private final SendableChooser<Double> elevatorHeight = new SendableChooser<>();
    private double height = 2.0;
    // Sim vars
    public final Mechanism2d elevMech = new Mechanism2d(6.0, 12.0);
    private final MechanismRoot2d elevRoot = elevMech.getRoot("elevRoot", 3.0, 0.0);
    public final MechanismLigament2d elevatorViz = elevRoot
            .append(new MechanismLigament2d("ElevatorLigament", 10.0, 90, 70.0, new Color8Bit(Color.kOrange)));
    private final ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2), 12, 10, Units.inchesToMeters(1.5),
            0, 30,
            true, height);

    public boolean usePID = true;

    public ElevatorSubsystem() {
        if (Utils.isSimulation()) {
            elevatorFxLeader = new TalonFX(kElevatorLeaderId);
            elevatorFxFollower = new TalonFX(kElevatorFollowerId);
            elevatorController = new PIDController(1.0, 0, 0);
            elevatorFeedforward = new ElevatorFeedforward(0, 0.2698, 2.0, 2.0);
        } else {
            elevatorFxLeader = new TalonFX(kElevatorLeaderId, "kingCan");
            elevatorFxFollower = new TalonFX(kElevatorFollowerId, "Kingcan");
            elevatorController = new PIDController(0, 0, 0);
            elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0);
        }

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 12;//TODO check gearstack irl

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = elevatorFxLeader.getConfigurator().apply(cfg);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        SmartDashboard.putData("elevatorViz", elevMech);

    }

    

    // refrence commands
    public Command setElevatorHight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    public Command setMotor(double voltage) {
        return runEnd(() -> {
            usePID = false;
            elevatorFxLeader.setVoltage(calculatevoltage());
        },
                () -> {
                    elevatorFxLeader.setVoltage(0);
                    usePID = true;
                });

    }

    public double calculatevoltage() {
        if (Utils.isSimulation()) {
            return elevatorFeedforward.calculate(0) + elevatorController.calculate(height);
        } else {
            return elevatorFeedforward.calculate(elevatorEncoderValue.get(), 0)
                    + elevatorController.calculate(elevatorEncoderValue.get(), height);
        }
    }


    @Override
    public void periodic() {
        if (Utils.isSimulation()) {
            elevatorFxLeader.setVoltage(calculatevoltage());
            simEncoder = elevatorViz.getLength();
            elevatorSim.setInput(elevatorFxLeader.getMotorVoltage().getValueAsDouble());
            elevatorSim.update(0.01);
            elevatorViz.setLength(elevatorSim.getPositionMeters());
            SmartDashboard.putNumber("height", height);
            SmartDashboard.putNumber("heightEncoder", elevatorEncoderValue.get());
            SmartDashboard.putNumber("Elevator Encoder", simEncoder);
            SmartDashboard.putNumber("Elevator set voltage", elevatorFxLeader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator follower set voltage",
                    elevatorFxFollower.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Height", height);
            SmartDashboard.putNumber("sim elev position", elevatorSim.getPositionMeters());
            SmartDashboard.putNumber("sim elev motor position", elevatorFxLeader.getPosition().getValueAsDouble());
        } else {

        }

    }

}
