package org.firstinspires.ftc.teamcode;

public class LocationPoint {
    public double x;
    public double y;

    public LocationPoint(double in_x, double in_y) {
        x = in_x;
        y = in_y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX(double x) {this.x = x;}

    public void setY(double y) {this.y = y;}

    public double getDistanceTo(LocationPoint other_point) {
        double deltaX = other_point.getX() - this.getX();
        double deltaY = other_point.getY() - this.getY();
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        return distance;
    }

    public double getAngleTo(LocationPoint other_point) {
        double deltaX = other_point.getX() - this.getX();
        double deltaY = other_point.getY() - this.getY();
        double theta = Math.atan2(deltaY, deltaX);
        return theta;
    }
}