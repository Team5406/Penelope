package ca._5406.util;

public class Util {
  private Util(){}
  
  public static double limit(double value, double limit){
	  if(Math.abs(value) > Math.abs(limit)) return Math.signum(value) * limit;
	  else return value;
  }
  
  public static double applyDeadband(double value, double deadband){
	  if(Math.abs(value) < Math.abs(deadband)) return 0.0;
	  else return value;
  }
}