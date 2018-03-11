package ca._5406.penelope;

import ca._5406.util.BrownoutMonitor;
import ca._5406.util.CurrentLimiter;
import ca._5406.util.Looper;
import ca._5406.util.Motors;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive implements Looper.Loopable {
  
  private static final Drive drivetrain = new Drive();
  
  private static final double HIGH_GEAR_RATIO = 7.08;
  private static final double LOW_GEAR_RATIO = 26.04;
  
  private DoubleSolenoid shiftSolenoid = new DoubleSolenoid(0, 1);
  private NeutralMode lastNeutralMode = NeutralMode.Coast;
  private WPI_TalonSRX leftMaster = initMasterTalon(1, true);
  private WPI_TalonSRX leftFollower1 = initFollowerTalon(2, leftMaster);
  private WPI_TalonSRX leftFollower2 = initFollowerTalon(3, leftMaster);
  private WPI_TalonSRX rightMaster = initMasterTalon(4, false);
  private WPI_TalonSRX rightFollower1 = initFollowerTalon(5, rightMaster);
  private WPI_TalonSRX rightFollower2 = initFollowerTalon(6, rightMaster);
  private CurrentLimiter currentLimiter = new CurrentLimiter(Motors.CIM, 50);
  private BrownoutMonitor brownoutMonitor = new BrownoutMonitor(Motors.CIM, 3, 8);
  
  private double desiredLeftPower = 0;
  private double desiredRightPower = 0;
  private boolean currentLimitingEnabled = false;
  
  private Drive(){
    shiftSolenoid.set(Gear.LOW.value);
  }
  
  public static Drive getInstance(){
    return drivetrain;
  }
  
  @Override
  public void update(){
    double leftSetpoint = desiredLeftPower;
    double rightSetpoint = desiredRightPower;
    
    if(currentLimitingEnabled){
      double leftVoltage = leftMaster.getMotorOutputVoltage();
      double rightVoltage = rightMaster.getMotorOutputVoltage();
      double leftSpeed = encVelToRpm(leftMaster.getSelectedSensorVelocity(0));
      double rightSpeed = encVelToRpm(rightMaster.getSelectedSensorVelocity(0));
      
      double brownoutScaleFactor = brownoutMonitor.getScalingFactor(leftVoltage, leftSpeed, rightVoltage, rightSpeed, isHighGear());
      double leftCurrentFactor = currentLimiter.getScalingFactor(leftVoltage * brownoutScaleFactor, leftSpeed);
      double rightCurrentFactor = currentLimiter.getScalingFactor(rightVoltage * brownoutScaleFactor, rightSpeed);
      double scaleFactor = Math.max(Math.max(leftCurrentFactor, rightCurrentFactor), brownoutScaleFactor);
      
      leftSetpoint *= scaleFactor;
      rightSetpoint *= scaleFactor;
      SmartDashboard.putNumber("current_scale_factor", scaleFactor);
    }
    else{
        SmartDashboard.putNumber("current_scale_factor", 1.0);
    }
    
    leftMaster.set(leftSetpoint);
    rightMaster.set(rightSetpoint);
  }
  
  public void setGear(Gear desiredGear){
    shiftSolenoid.set(desiredGear.value);
  }
  
  public boolean isHighGear(){
    return shiftSolenoid.get() == Gear.HIGH.value;
  }
  
  public void setCurrentLimitingEnabled(boolean enabled){
    currentLimitingEnabled = enabled;
  }
  
  public double encVelToRpm(double vel){
    return vel * (1.0 / 4096 * 2 * Math.PI * (isHighGear() ? HIGH_GEAR_RATIO : LOW_GEAR_RATIO));
  }
  
  public void driveLeftRight(double left, double right){
    desiredLeftPower = left;
    desiredRightPower = right;
  }
  
  public void doArcadeDrive(double throttle, double turn){
    desiredLeftPower = throttle + turn;
    desiredRightPower = throttle - turn;
  }
  
  public void setNeutralMode(NeutralMode newMode){
    if(newMode != lastNeutralMode){
      leftMaster.setNeutralMode(newMode);
      leftFollower1.setNeutralMode(newMode);
      leftFollower2.setNeutralMode(newMode);
      rightMaster.setNeutralMode(newMode);
      rightFollower1.setNeutralMode(newMode);
      rightFollower2.setNeutralMode(newMode);
      lastNeutralMode = newMode;
    }
  }
  
  private WPI_TalonSRX initMasterTalon(int deviceNumber, boolean invert){
    WPI_TalonSRX _talon = new WPI_TalonSRX(deviceNumber);
    _talon.setInverted(invert);
    _talon.setNeutralMode(lastNeutralMode);
    _talon.configOpenloopRamp(0.5, 0);
    _talon.configContinuousCurrentLimit(40, 0);
    _talon.configPeakCurrentLimit(60, 0);
    _talon.configPeakCurrentDuration(50, 0);
    _talon.enableCurrentLimit(true);
    _talon.configNominalOutputForward(0, 0);
    _talon.configNominalOutputReverse(-0, 0);
    _talon.configPeakOutputForward(1, 0);
    _talon.configPeakOutputReverse(-1, 0);
    _talon.configVoltageCompSaturation(12, 0);
    _talon.enableVoltageCompensation(true);
    _talon.set(ControlMode.PercentOutput, 0.0);
    return _talon;
  }
  
  private WPI_TalonSRX initFollowerTalon(int deviceNumber, WPI_TalonSRX talonToFollow){
    WPI_TalonSRX _talon = new WPI_TalonSRX(deviceNumber);
    _talon.setInverted(talonToFollow.getInverted());
    _talon.setNeutralMode(lastNeutralMode);
    _talon.configOpenloopRamp(0, 0);
    _talon.set(ControlMode.Follower, talonToFollow.getDeviceID());
    return _talon;
  }
  
  public enum Gear {
    HIGH(DoubleSolenoid.Value.kReverse),
    LOW(DoubleSolenoid.Value.kForward);
    
    DoubleSolenoid.Value value;
    
    Gear(DoubleSolenoid.Value value){
      this.value = value;
    }
  }
}
