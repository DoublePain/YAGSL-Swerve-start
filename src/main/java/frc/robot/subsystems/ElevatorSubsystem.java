package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.Constants.AlgaeArmConstants;
import edu. wpi. first. math. trajectory. TrapezoidProfile.Constraints;
import frc.robot.RobotMath.Elevator;
import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
    //setUp
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);
    private final SparkMax m_motor = new SparkMax(Constants.IDConstants.Elevator_Left_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, m_elevatorGearbox);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.Kelevatorkp,
            ElevatorConstants.Kelevatorki,
            ElevatorConstants.Kelevatorkd,
            new Constraints(ElevatorConstants.kMaxVelocity,
                    ElevatorConstants.KMaxAccel));

                    private final DCMotor m_elevatorGearbox2 = DCMotor.getNEO(1);
                    private final SparkMax m_motor2 = new SparkMax(Constants.IDConstants.Elevator_Left_ID, SparkLowLevel.MotorType.kBrushless);
                    private final SparkMaxSim m_motorSim2 = new SparkMaxSim(m_motor, m_elevatorGearbox);
                    private final RelativeEncoder m_encoder2 = m_motor.getEncoder();
                    private final ProfiledPIDController m_controller2 = new ProfiledPIDController(ElevatorConstants.Kelevatorkp,
                            ElevatorConstants.Kelevatorki,
                            ElevatorConstants.Kelevatorkd,
                            new Constraints(ElevatorConstants.kMaxVelocity,
                                    ElevatorConstants.KMaxAccel));
                                    
    private final ElevatorFeedforward m_feedForward = new ElevatorFeedforward(ElevatorConstants.kElevatorS,
            ElevatorConstants.kElevatorG,
            ElevatorConstants.kElevatorV,
            ElevatorConstants.kElevatorA);
    private ElevatorSim m_elevatorSim = null;
    // Sensors
   
    public ElevatorSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40)
                .openLoopRampRate(ElevatorConstants.kElevatorRampRate);

        m_motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);//

        if (RobotBase.isSimulation()) {
            m_elevatorSim = new ElevatorSim(m_elevatorGearbox,
                    ElevatorConstants.kElevatorGearing,
                    ElevatorConstants.kElevatorCarriageMass,
                    ElevatorConstants.kElevatorDrumRadius,
                    ElevatorConstants.kElevatorMinHeight,
                    ElevatorConstants.kElevatorMaxHeight,
                    true,
                    0.0,
                    0.02,
                    0.0);

       
        }


    }

    public void simulationPeriodic() {
        //set input(voltage)
        m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        //update-every 20 milliseconds
        m_elevatorSim.update(0.02);

        m_motorSim.iterate(Elevator.convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond()))
                        .per(Second).in(RPM),
                RoboRioSim.getVInVoltage(),
                0.020);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

      
    }

    public double getPositionMeters() {
        return m_encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public double getVelocityMetersPerSecond() {
        return (m_encoder.getVelocity() / 60) * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public void reachGoal(double goal){
        double voltsOutput = MathUtil.clamp(
                m_feedForward.calculateWithVelocities(getVelocityMetersPerSecond(), m_controller.getSetpoint().velocity)
                + m_controller.calculate(getPositionMeters(), goal),
                -7,
                7);
        m_motor.setVoltage(voltsOutput);
    }

    public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public Command setElevatorHeight(double height){
        return setGoal(height).until(()->aroundHeight(height));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

 
    public void stop()
    {
        m_motor.set(0.0);
    }



}