package frc.robot.subsystems;


import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
    

  private CANSparkMax m_leftClimber;
  private CANSparkMax m_rightClimber;

  private boolean m_climbersRunning;

    public ClimberSubsystem() {
    m_leftClimber =
        new CANSparkMax(Constants.Climbers.KleftClimberID, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_leftClimber.setInverted( true);
    m_leftClimber.setSmartCurrentLimit(Constants.Climbers.kCurrentLimit);
    m_leftClimber.setIdleMode(IdleMode.kBrake);

    m_leftClimber.burnFlash();

    m_rightClimber =
        new CANSparkMax(Constants.Climbers.KrightClimberID, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rightClimber.setInverted(false );
    m_rightClimber.setSmartCurrentLimit(Constants.Climbers.kCurrentLimit);
    m_rightClimber.setIdleMode(IdleMode.kBrake);

    m_rightClimber.burnFlash();

    m_climbersRunning = false;
    }

    public void runClimber() {
        m_climbersRunning = true;
      }
      
    
    public void runMotor(double x){
        m_leftClimber.set(x);
        m_rightClimber.set(x);
    }
    
  /**
   * Turns the climbers off. Can be run once and the climbers will stay running or run continuously
   * in a {@code RunCommand}.
   */
    public void stopClimber() {
        m_climbersRunning = false;
      }
    
      @Override
      public void periodic() { // this method will be called once per scheduler run
        // set the climber motor powers based on whether the climber is on or not
        if (m_climbersRunning) {
          m_leftClimber.set(Constants.Climbers.kLeftPower);
          m_rightClimber.set(Constants.Climbers.kRightPower);
        } else {
          m_leftClimber.set(0.0);
          m_rightClimber.set(0.0);
        }
      }
    }

