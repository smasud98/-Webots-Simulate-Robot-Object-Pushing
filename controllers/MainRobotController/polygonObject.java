import java.util.ArrayList;
import java.awt.geom.Point2D;

/**
 * A polygon object.
 */
public class polygonObject{
    private ArrayList<Point2D.Double> edgePoints;   // collection of points that make the boundry of the object, in order around object
    public Point2D.Double center;                   // center that doesn't change, stays relative to edgePoints
    
    /**
     * Constructor
     * @param seed List of points in order around the perimeter of the object (clockwise or counter-closckwise).
     * @param centroid The center of the object.
     */
    public polygonObject(ArrayList<Point2D.Double> seed, Point2D.Double centroid){
        edgePoints = new ArrayList<Point2D.Double>();
        for(int i=0; i<seed.size(); i++) edgePoints.add(seed.get(i));
        center = new Point2D.Double(centroid.x, centroid.y);
    }

    /**
     * Rotates the object by a specified angle.
     * @param angle The angle to rotate the object by (degrees).
     */
    public void rotate(double angle){
        //cos and sin of angle done outside of loop to lower time complexity
        double tempCos=Math.cos(Math.toRadians(angle)),
                tempSin=Math.sin(Math.toRadians(angle));
        for(int x=0; x<edgePoints.size(); x++){
            double  tempX = edgePoints.get(x).x,
                    tempY = edgePoints.get(x).y;
            //2d rotation matrix
            edgePoints.get(x).x = (((tempX-center.x) * tempCos) - ((tempY-center.y) * tempSin)) + center.x;
            edgePoints.get(x).y = (((tempX-center.x) * tempSin) + ((tempY-center.y) * tempCos)) + center.y;
        }
    }

    /**
     * Checks if the given point is inside the polygonObject or along its perimeter. Done using a line line test.
     * @param checkThis Point to check.
     * @return Boolean value indicating if the point is within the PolygonObject.
     */
    public boolean contains(Point2D.Double checkThis) {
        //starting false equivalent to odd=false
        boolean result = false;
        for (int i=0; i<edgePoints.size(); i++){
            int j=i-1;
            if(i==0) j=edgePoints.size()-1;
            //return true if point lies on the perimeter
            if(checkThis.distance(edgePoints.get(i)) + checkThis.distance(edgePoints.get(j)) == edgePoints.get(i).distance(edgePoints.get(j))) return true;
            // flip result for every line passed
            if ((edgePoints.get(i).y > checkThis.y) != (edgePoints.get(j).y > checkThis.y) &&
                (checkThis.x < (edgePoints.get(j).x-edgePoints.get(i).x) * (checkThis.y-edgePoints.get(i).y) / (edgePoints.get(j).y-edgePoints.get(i).y) + edgePoints.get(i).x)) result = !result;
        }
        return result;
    }

    /**
     * Shifts the polygon so the new location is it's center point
     * @param location the new center point
     */
    public void shift(Point2D.Double location){
        center.x = location.x - center.x;
        center.y = location.y - center.y;
        for(int i=0; i<edgePoints.size(); i++){
            edgePoints.get(i).x +=center.x;
            edgePoints.get(i).y +=center.y;
        }
        center.x = location.x;
        center.y = location.y;
    }

    /**
     * Checks if the given line intersects with polygonObject.
     * @param checkThis1 starting point of line to check.
     * @param checkThis2 ending point of line to check.
     * @return Boolean value indicating if the point is within the PolygonObject.
     */
    public boolean containsLine(Point2D.Double checkThis1, Point2D.Double checkThis2) {
        double x1=checkThis1.x, y1=checkThis1.y,
               x2=checkThis2.x, y2=checkThis2.y;
        for (int i=0; i<edgePoints.size(); i++){
            int j=i-1;
            if(i==0) j=edgePoints.size()-1;
            double x3=edgePoints.get(i).x, y3=edgePoints.get(i).y,
                   x4=edgePoints.get(j).x, y4=edgePoints.get(j).y;
            double den = ((x1-x2)*(y3-y4))-((y1-y2)*(x3-x4));
            if(den!=0){
                if(this.contains(new Point2D.Double(((((x1*y2)-(y1*x2))*(x3-x4))-((x1-x2)*((x3*y4)-(y3*x4))))/den, ((((x1*y2)-(y1*x2))*(y3-y4))-((y1-y2)*((x3*y4)-(y3*x4))))/den)))return true;
            }
        }
        return false;
    }
}
