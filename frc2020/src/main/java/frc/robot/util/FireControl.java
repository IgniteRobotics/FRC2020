package frc.robot.util;

/**
 * A simple state machine using a singleton pattern to help control the shooter.
 */
public class FireControl {

    //the instance itself
    private static FireControl _instance = null;

    //instance variables
    private long onTargetTimer = 0;    
    private int powerCells = 0;
    private boolean shooterVelocityOK = false;
    private boolean kickerVelocityOK = false;

    // these should be moved ton Constants, or set from elsewhere
    private static final long onTargetThresholdMillis = 100;
    private static final int maxPowerCells = 5;

    //private constructor
    private FireControl() {
        this.onTargetTimer = 0;
        this.powerCells = 0;        
    }

    //public factory
    public static FireControl getInstance(){
        if (null == _instance){
            _instance = new FireControl();
        }
        return _instance;

    }

    public void setOnTarget(boolean state){
        if(state){
            this.onTargetTimer = System.currentTimeMillis();
        } else {
            this.onTargetTimer = 0;
        }
    }

    public boolean isOnTarget(){
        if (this.onTargetTimer > 0 && System.currentTimeMillis() - onTargetTimer >= onTargetThresholdMillis){
            return true;
        } else {
            return false;
        }
    } 

    public void powerCellIn(){
        this.powerCells++;
    }

    public void loadPowerCells(int n){
        this.powerCells = n;
    }

    public void shotFired(){
        this.powerCells--;
        if (powerCells < 0) {powerCells = 0;}
    }

    public void setShooterVelocityStatus(boolean status){
        this.shooterVelocityOK = status;
    }

    public void setKickerVelocityStatus(boolean status){
        this.kickerVelocityOK = status;
    }

    public boolean readyToFire(){
        // TODO count power cells when we have sensors.
        return (isOnTarget() 
                && this.kickerVelocityOK
                && this.shooterVelocityOK
        );
    }

    public boolean readyToInake(){
        return this.powerCells < maxPowerCells;
    }
}
