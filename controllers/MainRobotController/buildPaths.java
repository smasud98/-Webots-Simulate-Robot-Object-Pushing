import java.util.ArrayList;
import java.util.Collections;
import java.awt.geom.Point2D;

/**
 * Djikstra algorithm for fastest route for a robot to get to its goal. Visibility graph is only 2 dimensions as rotating is not necessary (can add a third dimension to incorporate rotations later, however a 2d version should be kept for the robot).
 */
public class buildPaths{
    //visibility graph explanation, respectively: graph layer, horizontal position, vertical position
    private static ArrayList <ArrayList <visibilityGraphNode>> visibilityGraph;
    private static ArrayList<Point2D.Double> solution;
    //user input variables
    private int numNodes=17;                     //How many nodes for width and depth of graph
    private ArrayList<polygonObject> obstacles;
    private polygonObject movingObject;
    private double mapLength, mapWidth;
    private static Point2D.Double endPoint;

    /**
     * Quick instance for simple testing, parameters given by other files (from webots) simulated here. Prints solution and visibility graph to console.
     * @param args not used. Can be easily modified to take in users input for testing.
     */
    public static void main(String[] args){
        //Temp variable to populate data
        ArrayList<Point2D.Double> tempBoundary = new ArrayList<Point2D.Double>();
        //Build moving object
        tempBoundary.add(new Point2D.Double(1.05,1.3));
        tempBoundary.add(new Point2D.Double(1.05,1.2));
        tempBoundary.add(new Point2D.Double(0.95,1.2));
        tempBoundary.add(new Point2D.Double(0.95,1.3));
        polygonObject movingObject = new polygonObject(tempBoundary, new Point2D.Double(1,1.25));
        //Build Obstacle list
        tempBoundary.clear();
        tempBoundary.add(new Point2D.Double(1.3,1.65));
        tempBoundary.add(new Point2D.Double(1.3,1.55));
        tempBoundary.add(new Point2D.Double(0.7,1.55));
        tempBoundary.add(new Point2D.Double(0.7,1.65));
        ArrayList<polygonObject>  obstacles = new ArrayList<polygonObject>();
        obstacles.add(new polygonObject(tempBoundary, new Point2D.Double(1,1.6)));
        //Map Size
        double mapLength=2, mapWidth=2;
        //Goal
        Point2D.Double endPoint = new Point2D.Double(1,1.9);
       
        //launch algorithm with artificial data
        buildPaths test = new buildPaths(obstacles, mapLength, mapWidth);
        test.runDjikstra(movingObject, endPoint);
        System.out.println("\nBUILDPATHS.JAVA TEST\nGoing from ("+movingObject.center.x+", "+movingObject.center.y+") to ("+endPoint.x+", "+endPoint.y+").");

        //display visibility graph with node location
        System.out.println("\nVisibility graph (node location):");
        for(int a=0; a<visibilityGraph.size(); a++){
            for(int b=0; b<visibilityGraph.get(a).size(); b++){
                if(visibilityGraph.get(a).get(b).isReachable) System.out.print("("+visibilityGraph.get(a).get(b).nodeLocation.a + ", " + visibilityGraph.get(a).get(b).nodeLocation.b+") ");
                else System.out.print("(null) ");
            }
            //System.out.println("");
        }

        //display visibility graph with real location
        System.out.println("\nVisibility graph (true location):");
        for(int a=0; a<visibilityGraph.size(); a++){
            for(int b=0; b<visibilityGraph.get(a).size(); b++){
                if(visibilityGraph.get(a).get(b).isReachable) System.out.print("("+visibilityGraph.get(a).get(b).trueLocation.x + ", " + visibilityGraph.get(a).get(b).trueLocation.y+") ");
                else System.out.print("(  null  ) ");
            }
            //System.out.println("");
        }

        //display solution
        //System.out.println("\n"+solution.size()+" points in path:");
        for(int x=0; x<solution.size(); x++){
            //System.out.println("p" + (x+1) + ": (" + solution.get(x).x + ", " + solution.get(x).y + ")");
        }
    }

    /**
     * Constructor, also builds visibility graph.
     * @param obstacleList      List of immovable objects on the map.
     * @param lengthOfMap       Length of the map.
     * @param widthOfMap        Width of the map.
     */
    public buildPaths(ArrayList<polygonObject> obstacleList, double lengthOfMap, double widthOfMap){
        //TODO - for 3D version (rotations), object being moved should be here for visibility graph, not in 'runDjikstra'
        visibilityGraph = new ArrayList<ArrayList<visibilityGraphNode>>();
        obstacles = new ArrayList<polygonObject>();
        obstacles = obstacleList;
        mapLength = lengthOfMap;
        mapWidth = widthOfMap;
        solution = null;
    
        buildVisibilityGraph();
    }

    /**
     * Runs djikstra algorithm from the objects starting point to the end location.
     * @param objectToBeMoved   Object being moved.
     * @param endCoordinate     Coordinates of target location.
     * @param extraObjects      not null when an extra object needs to be considered as a boundary object
     * @return                  Fastest path from object to target location.
     */
    public ArrayList<Point2D.Double> runDjikstra(polygonObject objectToBeMoved, Point2D.Double endCoordinate) {
        //TODO - when implementing rotations, 'runDjikstra' should have a 2D form for robots and 3D form for object
        //build new solution
        solution = new ArrayList<Point2D.Double>();
        movingObject = objectToBeMoved;
        endPoint = endCoordinate;

        //build list of potential paths
        ArrayList<ArrayList<TwoInt>> paths = new ArrayList<ArrayList<TwoInt>>();

        //find and set up starting node
        TwoInt start = new TwoInt(0,0);
        for(int x=0; x<visibilityGraph.size(); x++){
            for(int y=0; y<visibilityGraph.get(0).size(); y++){
                if(movingObject.center.distance(start.getCoords()) > movingObject.center.distance(getNode(x, y).trueLocation)) start.newSpot(x, y);
            }
        }
        getNode(start).distanceSoFar=0;
        paths.add(new ArrayList<TwoInt>());
        paths.get(0).add(start);
        
        //find ending node
        TwoInt end = new TwoInt(0,0);
        for(int x=0; x<visibilityGraph.size(); x++){
            for(int y=0; y<visibilityGraph.get(0).size(); y++){
                if(endPoint.distance(end.getCoords()) > endPoint.distance(getNode(x, y).trueLocation)) end.newSpot(x, y);
            }
        }

        //algorithm loop
        loop:
        for(;;){
            //pointer to the node with shortest path (allways first in list)
            TwoInt lastPosition = paths.get(0).get(paths.get(0).size()-1);
            visibilityGraphNode last = getNode(lastPosition);

            //for each connecting node build new path options
            boolean isNewNodes=false;
            //add new paths, calculate distance, and remove extra path if new node distance != -1
            isNewNodes=makeNewPath(paths, last, lastPosition.a+1, lastPosition.b);
            isNewNodes=makeNewPath(paths, last, lastPosition.a-1, lastPosition.b);
            isNewNodes=makeNewPath(paths, last, lastPosition.a, lastPosition.b+1);
            isNewNodes=makeNewPath(paths, last, lastPosition.a, lastPosition.b-1);
            
            //if new nodes have been made
            if(isNewNodes){
                //delete first path (old path)
                paths.remove(0);

                // check if goal is reached
                for(int a=0; a<paths.size(); a++){
                    if(paths.get(a).get(paths.get(a).size()-1).a == end.a && paths.get(a).get(paths.get(a).size()-1).b == end.b){
                        Collections.swap(paths, 0, a);
                        break loop;
                    }
                }

                //find smallest 'distanceSoFar' and move to front of list
                for(int i=1; i<paths.size(); i++){
                    visibilityGraphNode shortestSoFar = getNode(paths.get(0).get(paths.get(0).size()-1));
                    visibilityGraphNode checking = getNode(paths.get(i).get(paths.get(i).size()-1));
                    if(shortestSoFar.distanceSoFar > checking.distanceSoFar){
                        Collections.swap(paths, i, 0);
                    }
                }
            }else{
                //no new paths, shortest path stuck, delete old path if size is bigger than 0
                if(paths.size()>1) paths.remove(0);
                //out of paths, increase numnodes, rebuild visibility graph and start over
                else{
                    numNodes*=2;
                    buildVisibilityGraph();
                    paths.clear();
                    paths.add(new ArrayList<TwoInt>());
                    start = new TwoInt(0,0);
                    for(int x=0; x<visibilityGraph.size(); x++){
                        for(int y=0; y<visibilityGraph.get(0).size(); y++){
                            if(movingObject.center.distance(start.getCoords()) > movingObject.center.distance(getNode(x, y).trueLocation)) start.newSpot(x, y);
                        }
                    }
                    paths.get(0).add(start);
                }
            }
        }
        
        //build solution
        for(int i=0; i<paths.get(0).size(); i++) solution.add(paths.get(0).get(i).getCoords());
        //final point
        //if(solution.get(solution.size()-1).x != endPoint.x || solution.get(solution.size()-1).y != endPoint.y) solution.add(new Point2D.Double(endPoint.x, endPoint.y));
        return solution;
    }

    /**
     * 
     * @param location
     * @return
     */
    public Point2D.Double getClosestNode(Point2D.Double location){
        TwoInt closestPoint = new TwoInt(0,0);
        for(int x=0; x<visibilityGraph.size(); x++){
            for(int y=0; y<visibilityGraph.get(0).size(); y++){
                if(location.distance(closestPoint.getCoords()) > location.distance(getNode(x, y).trueLocation)) closestPoint.newSpot(x, y);
            }
        }
        return closestPoint.getCoords();
    }

    /**
     * ONLY USE WHEN OBJECT BEING PUSHED IS STORED AS OBSTACLES LAST INDEX
     */
    public void updatePushingObject(Point2D.Double location){
        obstacles.get(obstacles.size()-1).shift(location);
        buildVisibilityGraph();
    }

    /**
     * Attemps to add a new path to 'paths', returns true if successful.
     * @param paths List of potential paths for solution
     * @param last old node, going to new node
     * @param newA new nodes a position
     * @param newB new nodes b position
     * @return Value indicating if new path was successfully built
     */
    private boolean makeNewPath(ArrayList<ArrayList<TwoInt>> paths, visibilityGraphNode last, int newA, int newB){
        //Make sure new node is reachable
        if(newA>=0 && newA<visibilityGraph.size() && newB>=0 && newB<visibilityGraph.get(0).size()){
            if(getNode(newA, newB).isReachable){
                //checks if the path to the next node is passable
                for(int i=0; i<obstacles.size(); i++) if(obstacles.get(i).containsLine(last.trueLocation, getNode(newA, newB).trueLocation)) return false;

                //make new path and add to list (at end)
                visibilityGraphNode next = getNode(newA, newB);
                ArrayList<TwoInt> newPath = new ArrayList<TwoInt>();
                for(int i=0; i<paths.get(0).size();i++) newPath.add(paths.get(0).get(i).copy());
                newPath.add(next.nodeLocation);
                paths.add(newPath);

                // calculate 'distanceSoFar'
                double newDist = last.distanceSoFar + Math.abs(last.trueLocation.distance(next.trueLocation));
                //if 'distanceSoFar' is not empty, delete longer path
                if (next.distanceSoFar!=-1){
                    if(newDist>next.distanceSoFar){
                        paths.remove(paths.size()-1);
                        //New path failed, return false
                        return false;
                    }else{
                        for(int i=paths.size()-1; i>=0; i--){
                            if(paths.get(i).get(paths.get(i).size()-1).a == paths.get(paths.size()-1).get(paths.get(paths.size()-1).size()-1).a &&
                            paths.get(i).get(paths.get(i).size()-1).b == paths.get(paths.size()-1).get(paths.get(paths.size()-1).size()-1).b &&
                            i!=paths.size()-1){
                                paths.remove(i);
                                next.distanceSoFar=newDist;
                            }
                        }
                    }
                }else{
                    //if 'distanceSoFar' is empty, use new distance
                    next.distanceSoFar=newDist;
                }
                return true;
            }
        }
        return false;
    }

    /**
     * Calculates visibility graph and stores it in 'visibilityGraph'.
     */
    private void buildVisibilityGraph(){
        double xDist=mapLength/(numNodes-1);
        double yDist=mapWidth/(numNodes-1);
        //build new empty graph
        visibilityGraph.clear();
        for(int l=0; l<numNodes; l++){
            visibilityGraph.add(new ArrayList<visibilityGraphNode>());
            for(int w=0; w<numNodes; w++){
                //build individual nodes
                visibilityGraph.get(l).add(new visibilityGraphNode(xDist*l, yDist*w, l, w));
                //check if any point has an obstacle in it
                for(int i=0; i<obstacles.size(); i++){
                    if(obstacles.get(i).contains(getNode(l, w).trueLocation)) getNode(l, w).isReachable = false;

                }
            }
        }
    }

    /**
     * Returns desired node from visibility graph
     * @param a first index into the visibility graph
     * @param b second index into the visibility graph
     * @return desired node from visibility graph
     */
    private visibilityGraphNode getNode(int a, int b){
        return visibilityGraph.get(a).get(b);
    }

    /**
     * Returns desired node from visibility graph
     * @param input location in the visibility graph
     * @return desired node from visibility graph
     */
    private visibilityGraphNode getNode(TwoInt input){
        return visibilityGraph.get(input.a).get(input.b);
    }

    /**
     * Information for individual nodes within the visibility graph.
     */
    private static class visibilityGraphNode{
        private Point2D.Double trueLocation;    // Location of the node on the actual map
        private TwoInt nodeLocation;            // Position of the node in the visibility graph
        private double distanceSoFar;           // Distance traveled so far up to this node, initializes as '-1'
        private boolean isReachable;            // Indicates if the node is reachable

        /**
         * Constructor
         * @param x x coordinate of the node on the actual map
         * @param y y coordinate of the node on the actual map
         * @param a first index into its position in the visibility graph
         * @param b second index into its position in the visibility graph
         */
        private visibilityGraphNode(double x, double y, int a, int b){
            trueLocation = new Point2D.Double();
            trueLocation.x = x;
            trueLocation.y = y;
            nodeLocation = new TwoInt(a, b);
            distanceSoFar = -1;
            isReachable = true;
        }
    }

    /**
     * Simple structure for indexing into the visibility graph.
     */
    private static class TwoInt{
        private int a,b; //positions in the visibility graph

        /**
         * Constructor
         * @param one first index into the visibility graph
         * @param two second index into the visibility graph
         */
        private TwoInt(int one, int two){
            a=one;
            b=two;
        }

        /**
         * Returns the coordinates of the given node in the visibility graph.
         * @return The coordinates of the given node in the visibility graph.
         */
        private Point2D.Double getCoords(){
            return visibilityGraph.get(a).get(b).trueLocation;
        }

        /**
         * Switches the node being looked at 
         * @param one new nodes first index into the visibility graph
         * @param two new nodes second index into the visibility graph
         */
        private void newSpot(int one, int two){
            a=one;
            b=two;
        }

        /**
         * Makes a copy of the current TwoInt
         * @return A copy of the current TwoInt.
         */
        private TwoInt copy(){
            return new TwoInt(a, b);
        }
    }
}