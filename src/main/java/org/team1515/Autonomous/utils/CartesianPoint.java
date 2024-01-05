package org.team1515.Autonomous.utils;

import java.util.function.DoubleFunction;

public class CartesianPoint {
    public Double x;
    public Double y;
    public CartesianPoint(Double x, Double y){
        this.x = x;
        this.y = y;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
}
