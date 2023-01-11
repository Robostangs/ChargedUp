package frc.robot;

public class Utils {
    
    public static double degToRad(double x) {
        return (Math.PI / 180.0) *x;
    }

    public static double customDeadzone(double input) {
        if(Math.abs(input) >= Constants.Drivetrain.CustomDeadzone.kLowerLimitExpFunc && Math.abs(input) < Constants.Drivetrain.CustomDeadzone.kUpperLimitExpFunc) {
            return Math.signum(input) * ((Constants.Drivetrain.CustomDeadzone.kExpFuncMult) * Math.pow((Constants.Drivetrain.CustomDeadzone.kExpFuncBase), Math.abs(input)) - Constants.Drivetrain.CustomDeadzone.kExpFuncConstant); 
        } else if(Math.abs(input) >= Constants.Drivetrain.CustomDeadzone.kUpperLimitExpFunc && Math.abs(input) <= Constants.Drivetrain.CustomDeadzone.kUpperLimitLinFunc) {
            return Math.signum(input) * ((Constants.Drivetrain.CustomDeadzone.kLinFuncMult) * (Math.abs(input) - Constants.Drivetrain.CustomDeadzone.kLinFuncOffset) + (Constants.Drivetrain.CustomDeadzone.kLinFuncConstant));
        }
        return Constants.Drivetrain.CustomDeadzone.kNoSpeed;
    }

}
