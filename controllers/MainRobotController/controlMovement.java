import java.util.ArrayList;
import java.awt.geom.Point2D;

public class controlMovement{
    polygonObject objectToBeMoved, robot;
    ArrayList<polygonObject> obstacleList;
    double lengthOfMap, widthOfMap;
    Point2D.Double end;
    double objectDiameter;


    private static ArrayList<Point2D.Double> robotSolution;
    private static ArrayList<Point2D.Double> objectSolution;

    /**
     * Quick instance for simple testing, parameters given by other files (from webots) simulated here.
     */
    public static void main(String[] args){
        //Temp variable to populate data
        ArrayList<Point2D.Double> tempBoundary = new ArrayList<Point2D.Double>();
        //Build moving object
        tempBoundary.add(new Point2D.Double(1.05,0.7));
        tempBoundary.add(new Point2D.Double(1.05,0.8));
        tempBoundary.add(new Point2D.Double(0.95,0.8));
        tempBoundary.add(new Point2D.Double(0.95,0.7));
        polygonObject movingObject = new polygonObject(tempBoundary, new Point2D.Double(1,0.75));
        double objSize = 0.1;
        //Build robot
        tempBoundary.clear();
        tempBoundary.add(new Point2D.Double(1.037, 1.037));
        tempBoundary.add(new Point2D.Double(1.037, 0.963));
        tempBoundary.add(new Point2D.Double(0.963, 0.963));
        tempBoundary.add(new Point2D.Double(0.963, 1.037));
        polygonObject  rowBot = new polygonObject(tempBoundary, new Point2D.Double(1,1));
        //Map Size
        double mapLength=2, mapWidth=2;
        //Goal
        Point2D.Double endPoint = new Point2D.Double(0.3,0.3);

        //populate data
        new controlMovement(movingObject, rowBot, new ArrayList<polygonObject>(), mapLength, mapWidth, endPoint, objSize);

        //display test case solution
        System.out.println("   ROBOT     |  OBJECT");
        System.out.println("   "+robotSolution.size()+"        |  "+objectSolution.size());
        for(int i=0; i<robotSolution.size(); i++){
            String line = i+" ("+robotSolution.get(i).x + ", " + robotSolution.get(i).y +")";
            if(i<objectSolution.size()){
                line += " | ("+objectSolution.get(i).x + ", " + objectSolution.get(i).y +")";
            }
            System.out.println(line);
        }
    }

    /**
     * 
     * @param objectToBeMoved
     * @param robot
     * @param obstacleList
     * @param lengthOfMap
     * @param widthOfMap
     * @param end
     */
    public controlMovement(polygonObject objectToBeMoved, polygonObject robot, ArrayList<polygonObject> obstacleList, double lengthOfMap, double widthOfMap, Point2D.Double end, double diameterOfObject){
        this.objectToBeMoved = objectToBeMoved;
        this.robot = robot;
        this.obstacleList = obstacleList;
        this.lengthOfMap = lengthOfMap;
        this.widthOfMap = widthOfMap;
        this.end = end;
        objectDiameter = diameterOfObject;

        update(objectToBeMoved, robot);
    }

    /**
     * When moveable object or robot diverge from intendended path, rebuild paths.
     * @param newObjectToBeMoved
     * @param newRobot
     */
    public void update(polygonObject newObjectToBeMoved, polygonObject newRobot){
        objectToBeMoved = newObjectToBeMoved;
        robot = newRobot;
        
        //build objects visibility graph
        buildPaths objectPathGenerator = new buildPaths(obstacleList, lengthOfMap, widthOfMap);
        //build objects path
        ArrayList<Point2D.Double> objectPath = objectPathGenerator.runDjikstra(objectToBeMoved, end);
        objectPath = simplify(objectPath);
        
        if(objectPath.get(0).x != objectToBeMoved.center.x || objectPath.get(0).y != objectToBeMoved.center.y) {
            objectPath.add(0, new Point2D.Double(objectToBeMoved.center.x, objectToBeMoved.center.y));
        }
        objectPath.add(new Point2D.Double(objectPath.get(objectPath.size()-1).x, end.y));
        objectPath.add(new Point2D.Double(end.x, end.y));


        //initialize robot path
        ArrayList<Point2D.Double> robotPath = new ArrayList<Point2D.Double>();
        robotPath.add(robot.center);
        
        //LOOP - adds object path one coordinate at a time, adjusting robots position first if needed
        for(int i=0; i+1<objectPath.size(); i++){
            //next point for robot
//TODO - this point might not be reachable
            Point2D.Double directionOfPush = new Point2D.Double(objectPath.get(i).x + objectPath.get(i).x - objectPath.get(i+1).x, objectPath.get(i).y + objectPath.get(i).y - objectPath.get(i+1).y);
            Point2D.Double nextRobotGoal = accountForObject(objectPath.get(i), directionOfPush);
//System.out.println("ADJUST TO OBJECT");
//System.out.println("Going to object point ("+objectPath.get(i).x+", "+objectPath.get(i).y+")");
//System.out.println("- robot adjustment, goes to ("+nextRobotGoal.x+", "+nextRobotGoal.y+"). Starting at index "+(robotPath.size()-1));


            //if not at next robot point, go there
//            for(;nextRobotGoal.x != robotPath.get(robotPath.size()-1).x || nextRobotGoal.y != robotPath.get(robotPath.size()-1).y;){
                ArrayList<Point2D.Double> tempList = new ArrayList<Point2D.Double>();
                objectToBeMoved.shift(objectPath.get(i));
                if(objectToBeMoved.contains(new Point2D.Double(robotPath.get(robotPath.size()-1).x, nextRobotGoal.y))){
                    //try one direction
                    tempList.add(new Point2D.Double(nextRobotGoal.x, robotPath.get(robotPath.size()-1).y));
                    tempList.add(new Point2D.Double(nextRobotGoal.x, nextRobotGoal.y));
                    if(!pathPossible(tempList)){
                        //try other direction
                        tempList.clear();
                        Point2D.Double temp = new Point2D.Double();
                        temp.x = robotPath.get(robotPath.size()-1).x + robotPath.get(robotPath.size()-1).x - nextRobotGoal.x;
                        temp.y = nextRobotGoal.y + nextRobotGoal.y - robotPath.get(robotPath.size()-1).y;
                        tempList.add(new Point2D.Double(temp.x, robotPath.get(robotPath.size()-1).y));
                        tempList.add(new Point2D.Double(temp.x, temp.y));
                        tempList.add(new Point2D.Double(nextRobotGoal.x, temp.y));
                        tempList.add(new Point2D.Double(nextRobotGoal.x, nextRobotGoal.y));
                    }
                }else{
                    //try one direction
                    tempList.add(new Point2D.Double(robotPath.get(robotPath.size()-1).x, nextRobotGoal.y));
                    tempList.add(new Point2D.Double(nextRobotGoal.x, nextRobotGoal.y));
                    if(!pathPossible(tempList)){
                        //try other direction
                        tempList.clear();
                        Point2D.Double temp = new Point2D.Double();
                        temp.x = nextRobotGoal.x + nextRobotGoal.x - robotPath.get(robotPath.size()-1).x;
                        temp.y = robotPath.get(robotPath.size()-1).y + robotPath.get(robotPath.size()-1).y - nextRobotGoal.y;
                        tempList.add(new Point2D.Double(robotPath.get(robotPath.size()-1).x, temp.y));
                        tempList.add(new Point2D.Double(temp.x, temp.y));
                        tempList.add(new Point2D.Double(temp.x, nextRobotGoal.y));
                        tempList.add(new Point2D.Double(nextRobotGoal.x, nextRobotGoal.y));
                    }
                }
                // if(!pathPossible(tempList)){
                //     //increase distance
                //     tempList.clear();
                //     Point2D.Double temp = new Point2D.Double();
                //     temp.x = nextRobotGoal.x + nextRobotGoal.x - objectPath.get(i).x;
                //     temp.y = nextRobotGoal.y + nextRobotGoal.y - objectPath.get(i).y;
                //     nextRobotGoal = accountForObject(nextRobotGoal, temp);
                //     temp.x = robotPath.get(robotPath.size()-1).x + robotPath.get(robotPath.size()-1).x - objectPath.get(i).x;
                //     temp.y = robotPath.get(robotPath.size()-1).y + robotPath.get(robotPath.size()-1).y - objectPath.get(i).y;
                //     robotPath.add(accountForObject(robotPath.get(robotPath.size()-1), temp));
                // }else{
                //     for(int a=0;a<tempList.size();a++) robotPath.add(tempList.get(a));
                // }
                if(!pathPossible(tempList)){
                    //increase distance
                    tempList.clear();
                    if(objectToBeMoved.contains(new Point2D.Double(robotPath.get(robotPath.size()-1).x, nextRobotGoal.y))){
                        tempList.add(new Point2D.Double(nextRobotGoal.x, robotPath.get(robotPath.size()-1).y));
                    }else{
                        tempList.add(new Point2D.Double(robotPath.get(robotPath.size()-1).x, nextRobotGoal.y));
                    }
                    tempList.add(nextRobotGoal);
                }
                for(int a=0;a<tempList.size();a++) robotPath.add(tempList.get(a));
//            }
            
            //robot is in correct position
            robotPath.add(accountForObject(objectPath.get(i+1), robotPath.get(robotPath.size()-1)));
//System.out.println("PUSH OBJECT");
//System.out.println("Going to object point ("+objectPath.get(i+1).x+", "+objectPath.get(i+1).y+")");
//System.out.println("- robot adjustment, goes to ("+robotPath.get(robotPath.size()-1).x+", "+robotPath.get(robotPath.size()-1).y+"). At index "+(robotPath.size()-1));

        }
        //output full robot path solution and object solution
        //robotPath = simplify(robotPath);
        robotSolution = robotPath;
        objectSolution = objectPath;
    }

    public boolean pathPossible(ArrayList<Point2D.Double> path){
        for(int a=0, b=path.size()-1; a<path.size(); b=a++) for(int i=0; i<obstacleList.size(); i++) if(obstacleList.get(i).containsLine(path.get(a), path.get(b))) return false;
        return true;
    }

    public boolean pathPossible(Point2D.Double point1, Point2D.Double point2){
        for(int i=0; i<obstacleList.size(); i++) if(obstacleList.get(i).containsLine(point1, point2)) return false;
        return true;
    }

    /**
     * Accounts for the size of the object
     * @param target the target location
     * @param fromDirection to determine direction
     * @return the new target
     */
    public Point2D.Double accountForObject(Point2D.Double target, Point2D.Double fromDirection){
        double relativeDistance =(objectDiameter)/Math.sqrt(((target.x-fromDirection.x)*(target.x-fromDirection.x)) + ((target.y-fromDirection.y)*(target.y-fromDirection.y)));
        Point2D.Double response = new Point2D.Double(((1-relativeDistance)*target.x)+(relativeDistance*fromDirection.x), ((1-relativeDistance)*target.y)+(relativeDistance*fromDirection.y));
        return response;
    }

    private ArrayList<Point2D.Double> simplify(ArrayList<Point2D.Double> path){
        ArrayList<Point2D.Double> newPath = new ArrayList<Point2D.Double>();
        newPath.add(path.get(0));

        for(int i=1; i+1<path.size(); i++){
            Point2D.Double vector1 = new Point2D.Double(path.get(i+1).x-path.get(i).x,
                                                        path.get(i+1).y-path.get(i).y);
            Point2D.Double vector2 = new Point2D.Double(path.get(i).x-newPath.get(newPath.size()-1).x,
                                                        path.get(i).y-newPath.get(newPath.size()-1).y);
            // if cross product is 0, skip that point
            double cp =(vector1.x*vector2.y)-(vector2.x*vector1.y);
            double dp =(vector1.x*vector2.x)+(vector2.x*vector1.x);
            if(cp!=0) newPath.add(path.get(i));
            else{
                //don't skip if opposite directions
                if(dp==-1) newPath.add(path.get(i));
            }
        }
        newPath.add(path.get(path.size()-1));
        return newPath;
    }

    public ArrayList<Point2D.Double> getRobotSolution(){
        ArrayList<Point2D.Double> solutionOffset = new ArrayList<Point2D.Double>();
        for(int i=0; i<robotSolution.size(); i++) solutionOffset.add(new Point2D.Double(robotSolution.get(i).x-(lengthOfMap/2), (widthOfMap/2)-robotSolution.get(i).y));
        return solutionOffset;
    }

    public ArrayList<Point2D.Double> getObjectSolution(){
        ArrayList<Point2D.Double> solutionOffset = new ArrayList<Point2D.Double>();
        for(int i=0; i<objectSolution.size(); i++) solutionOffset.add(new Point2D.Double(objectSolution.get(i).x-(lengthOfMap/2), (widthOfMap/2)-objectSolution.get(i).y));
        return solutionOffset;
    }
}