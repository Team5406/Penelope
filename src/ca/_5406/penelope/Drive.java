package ca._5406.penelope;

import ca._5406.util.BrownoutMonitor;
import ca._5406.util.CurrentLimiter;
import ca._5406.util.Looper;
import ca._5406.util.Motors;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive {
  
  private static final Drive drivetrain = new Drive();
  
  private static final double HIGH_GEAR_RATIO = 7.08;
  private static final double LOW_GEAR_RATIO = 15.32;
  
  private DoubleSolenoid shiftSolenoid = new DoubleSolenoid(0, 1);
  private NeutralMode lastNeutralMode = NeutralMode.Coast;
  private WPI_TalonSRX leftMaster = initMasterTalon(1, true);
  private WPI_TalonSRX leftFollower1 = initFollowerTalon(2, leftMaster);
  private WPI_TalonSRX leftFollower2 = initFollowerTalon(3, leftMaster);
  private WPI_TalonSRX rightMaster = initMasterTalon(4, false);
  private WPI_TalonSRX rightFollower1 = initFollowerTalon(5, rightMaster);
  private WPI_TalonSRX rightFollower2 = initFollowerTalon(6, rightMaster);
  public BrownoutMonitor brownoutMonitor = new BrownoutMonitor(Motors.CIM, 3, 10);
  
  private double currentScaleFactor = 1.0;
  
  private Drive(){
    shiftSolenoid.set(Gear.LOW.value);
  }
  
  public static Drive getInstance(){
    return drivetrain;
  }
  
  private void setLeftRight(double leftPower, double rightPower){
    if(encVelToRad(leftMaster.getSelectedSensorVelocity(0)) > 0 || encVelToRad(rightMaster.getSelectedSensorVelocity(0)) > 0) {
      double leftVoltage = leftPower * 12 * (leftMaster.getInverted() ? -1 : 1) * currentScaleFactor; //leftMaster.getMotorOutputVoltage() * (leftMaster.getInverted() ? -1 : 1);
      double rightVoltage = rightPower * 12 * (rightMaster.getInverted() ? 1 : -1) * currentScaleFactor; //rightMaster.getMotorOutputVoltage() * (rightMaster.getInverted() ? 1 : -1);
      double leftSpeed = encVelToRad(leftMaster.getSelectedSensorVelocity(0));
      double rightSpeed = encVelToRad(rightMaster.getSelectedSensorVelocity(0));
      
      currentScaleFactor = brownoutMonitor.getScalingFactor(leftVoltage, leftSpeed, rightVoltage, rightSpeed);
      
      leftPower *= currentScaleFactor;
      rightPower *= currentScaleFactor;
    }
    
    leftMaster.set(leftPower);
    rightMaster.set(rightPower);
  }
  
  public void setGear(Gear desiredGear){
    shiftSolenoid.set(desiredGear.value);
  }
  
  public boolean isHighGear(){
    return shiftSolenoid.get() == Gear.HIGH.value;
  }
  
  public double encVelToRad(double vel){
    return vel * 10 / 4096 * 2 * Math.PI * (isHighGear() ? HIGH_GEAR_RATIO : LOW_GEAR_RATIO);
  }
  
  public void driveLeftRight(double left, double right){
	  setLeftRight(left, right);
  }
  
  public void doArcadeDrive(double throttle, double turn){
    setLeftRight(throttle + turn, throttle - turn);
  }
  
  public double getCurrentScalingFactor(){
	  return currentScaleFactor;
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
    _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 50);
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
