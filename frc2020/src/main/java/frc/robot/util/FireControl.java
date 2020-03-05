/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class FireControl {

    //the instance itself
    private static FireControl _instance = null;

    //instance variables
    private long onTargetTimer = 0;    
    private int powerCells = 0;

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
        if (System.currentTimeMillis() - onTargetTimer >= onTargetThresholdMillis){
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

    public boolean readyToFire(){
        return (isOnTarget() && this.powerCells > 0);
    }

    public boolean readyToInake(){
        return this.powerCells < maxPowerCells;
    }
}
