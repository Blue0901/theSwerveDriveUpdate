package frc.robot;
public class ExtraMath {
    public static final double mod(double x, double y){
        double r = x % y;
        return x < 0 ? r += y : r;
    }
    
}
