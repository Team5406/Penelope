package ca._5406.util;

public class CurrentLimiter {
  private Motors motorType;
  
  private double currentLimit;
  
  public CurrentLimiter(Motors motorType, double currentLimit){
    this.motorType = motorType;
    this.currentLimit = currentLimit;
  }
  
  public double getScalingFactor(double voltage, double rpm){
    return Math.min(1.0, currentLimit / motorType.getEstimatedCurrent(voltage, rpm));
  }
  
}