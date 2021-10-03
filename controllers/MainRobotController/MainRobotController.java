import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;

import java.util.ArrayList;
import java.awt.geom.Point2D;


public class MainRobotController {
    
    public static double lengthOfMap = 2.0;
    public static double widthOfMap = 2.0;

    public static void main(String[] args) {
        
        final int TIME_STEP = 64;
        final double THRESH = 0.01; //0.005
        final double SPEED = 1.0;
        final double ePuckSize = 0.074;
        final Point2D.Double destination = new Point2D.Double(-0.8, 0.8);
        
        
        // EXTRACT FIELDS -------------------------------------------------------------------------------------
        Supervisor robot = new Supervisor();
        Point2D.Double robotCenter = new Point2D.Double(0.0, 0.0);
        
        
        // Extract fields of the moving object
        Node movingObject = robot.getFromDef("movingObject");            
        Field movingObjectSizeField = movingObject.getField("size");
        
        double movingObjectSize = movingObjectSizeField.getSFVec3f()[0];
        
        
        // Extract fields of Obstacle1
        Node o1Node = robot.getFromDef("o1");
        
        Field o1TranslationField = o1Node.getField("translation");
        Field o1SizeField = o1Node.getField("size");    
        
        double o1XSize = o1SizeField.getSFVec3f()[0];
        double o1YSize = o1SizeField.getSFVec3f()[2];
        
        // Extract fields of Obstacle2
        /*Node o2Node = robot.getFromDef("o2");
        
        Field o2TranslationField = o2Node.getField("translation");
        Field o2SizeField = o2Node.getField("size");    
        
        double o2XSize = o2SizeField.getSFVec3f()[0];
        double o2YSize = o2SizeField.getSFVec3f()[2];*/
               
        
        // Robot Compass
        Compass compass = robot.getCompass("compass");
        compass.enable(TIME_STEP);

        GPS gps = robot.getGPS("gps");
        gps.enable(TIME_STEP);

        Motor leftMotor = robot.getMotor("left wheel motor");
        Motor rightMotor = robot.getMotor("right wheel motor");
        
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        
        
        // CREATE POLYGONAL OBJECTS -----------------------------------------------------------------------------
        
        // The Robot
        
        Point2D.Double robotCenterOffset = Extensions.performOffset(robotCenter);       
        ArrayList<Point2D.Double> robotBounds = Extensions.getBoundaries(robotCenter, ePuckSize, ePuckSize);     
        polygonObject robotObjectPolygon = new polygonObject(robotBounds, robotCenterOffset);
        
        
        // The Box
        Point2D.Double movingObjectCenter = new Point2D.Double(0.0, 0.25);
        Point2D.Double movingObjectCenterOffset = Extensions.performOffset(movingObjectCenter);
        ArrayList<Point2D.Double> test = Extensions.getBoundaries(movingObjectCenter, movingObjectSize, movingObjectSize);
        polygonObject movingObjectPolygon = new polygonObject(test, movingObjectCenterOffset);
                
        
        // OBSTACLES
        
        ArrayList<polygonObject> obstacles = new ArrayList<polygonObject>();
        
        // Obstacle 1
        Point2D.Double o1Center = new Point2D.Double(0.0, 0.6);
        Point2D.Double o1CenterOffset = Extensions.performOffset(o1Center);
        ArrayList<Point2D.Double> o1Bounds = Extensions.getBoundaries(o1Center, o1XSize, o1YSize);        
        polygonObject o1Polygon = new polygonObject(o1Bounds, o1CenterOffset);
        
        obstacles.add(o1Polygon);
        
        
        // THE ALGORITHM - Run the Algorithm to get the shortest path --------------------------------------------
        
        controlMovement algorithm = new controlMovement(movingObjectPolygon, robotObjectPolygon, obstacles, 2.0, 2.0, Extensions.performOffset(destination), 0.1);
        ArrayList<Point2D.Double> path = algorithm.getRobotSolution();  
        ArrayList<Point2D.Double> objectPath = algorithm.getObjectSolution(); 
       
                
        
        // ROBOT MOVEMENT ----------------------------------------------------------------------------------------
        
        int pathIdx = 0;
        double[] temp,pos;
        Point2D.Double dest;
        double dx,dy,robotAngle,targetAngle,theta;
        boolean isDone = false;
        Point2D.Double point;

        
        // MAIN LOOP ---------------------------------------------------------------------------------------------
        
        while (robot.step(TIME_STEP) != -1) {
            //Point2D.Double movingObjectPos;
            
            // If the path is complete
            if(isDone){
                leftMotor.setVelocity(0);
                rightMotor.setVelocity(0);
            }
            else {
                
                //Field movingObjectTranslationField = movingObject.getField("translation");
                
                /*if (pathIdx == 0) {
                    movingObjectPos = new Point2D.Double(0.0, 0.25);
                } 
                else {
                    movingObjectPos = new Point2D.Double(movingObjectTranslationField.getSFVec3f()[0], 
                    movingObjectTranslationField.getSFVec3f()[2]);
                }*/
                
                            
                //Get Current Location
                temp = gps.getValues();
                pos = new double[]{temp[0], temp[2] * -1};
                dest = path.get(pathIdx);
                System.out.printf("dest: [%.2f, %.2f]\nGPS: (%.2f, %.2f)\n\n", dest.x, dest.y, pos[0], pos[1]);
                System.out.println(" ");
                dx = dest.x - pos[0];
                dy = dest.y - pos[1];
                

                // Check if we have reached the current point
                if (Math.abs(dx) < THRESH && Math.abs(dy) < THRESH)
                    ++pathIdx;

                // Check if final point reached
                if (pathIdx == path.size()) {
                    
                    //Destination Reached
                    System.out.println("Destination Reached!!!!");
                    isDone = true;
                }

                robotAngle = getRobotAngle(compass.getValues());
                targetAngle = getTargetAngle(dx, dy);
                theta = getTheta(robotAngle, targetAngle);
                System.out.printf("delta: [%.2f, %.2f] | theta: %.2f\n", dx, dy, theta);

                if (theta >= 2) {
                    
                    //Turn left
                    leftMotor.setVelocity(-SPEED);
                    rightMotor.setVelocity(SPEED);
                } 
                else if (theta > 1) {
                    
                    //Turn left
                    leftMotor.setVelocity(0.8 * SPEED);
                    rightMotor.setVelocity(SPEED);
                } 
                else if (theta > 0.1) {
                    
                    //Turn left
                    leftMotor.setVelocity(0.99 * SPEED);
                    rightMotor.setVelocity(SPEED);
                } 
                else if (theta > -1) {
                    
                    //Turn right
                    leftMotor.setVelocity(SPEED);
                    rightMotor.setVelocity(0.99 * SPEED);
                } 
                else if (theta > -2) {
                    
                    //Turn right
                    leftMotor.setVelocity(0.8 * SPEED);
                    rightMotor.setVelocity(SPEED);
                } 
                else {
                    leftMotor.setVelocity(SPEED);
                    rightMotor.setVelocity(-SPEED);
                }
                
                
                // Distance to Line
                /*System.out.println("MOVING OBJECT:  " + movingObjectPos.x + ", " + movingObjectPos.y);
                double distance = pointToLineDistance(movingObjectPos.x, movingObjectPos.y, objectPath.get(pathIdx).x, 
                    objectPath.get(pathIdx).y, objectPath.get(pathIdx+1).x, objectPath.get(pathIdx+1).y);
                    
                // If object goes off course
                /*if(distance > 0.05) {
                                  
                  // Re-Input Robot & Object data
                  /*robotObjectPolygon.shift(new Point2D.Double(pos[0], pos[1]));
                  movingObjectPolygon.shift(movingObjectPos);
                  
                  // Rereun the algorithm - Reset path, objectPath
                  algorithm.update(movingObjectPolygon, robotObjectPolygon);
                  path = algorithm.getRobotSolution();
                  objectPath = algorithm.getObjectSolution(); 
                  
                  // Reset pathIdx
                  pathIdx = 0;
                }*/
            }
        }
    }
    
    // Get the target angle
    private static double getTargetAngle(double dx, double dy) {

        double theta = Math.toDegrees(Math.atan2(dy, dx));

        if (theta<0)
            theta+=360;

        return theta;
    }
    
    // Get the angle of the robot
    private static double getRobotAngle(double[] cVals) {
        double rad = Math.atan2(cVals[0], cVals[2]);
        double bearing = (rad - 1.5708) / Math.PI * 180.0;
        if (bearing < 0.0)
            bearing += 360.0;
        bearing = 450-bearing;
        if (bearing > 360.0)
            bearing = bearing - 360.0;
        return bearing;
    }
    
    // Get the angle between the robot and target point
    private static double getTheta(double robotAngle, double targetAngle) {
        double theta = targetAngle - robotAngle;
        if (theta < -180)
            theta += 360;
        else if (theta > 180)
            theta -= 360;
        return theta;
    }
}