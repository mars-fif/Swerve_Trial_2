package frc.robot.util;

public class Anomath {
    public static double wrapAngle(double _angleRad){
        double twoPI = 2*Math.PI;

        if(_angleRad == twoPI){
            return 0.0;
        }
        else if(_angleRad > twoPI){
            return _angleRad - twoPI*Math.floor(_angleRad/twoPI);
        }
        else if(_angleRad < 0.0){
            return _angleRad + twoPI*(Math.floor((-_angleRad)/twoPI)+1);
        }
        else{
            return _angleRad;
        }
    }
}
