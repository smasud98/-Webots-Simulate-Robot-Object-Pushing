import java.util.ArrayList;
import java.awt.geom.Point2D;

public class Extensions {

	public static double lengthOfMap = 2.0;
  public static double widthOfMap = 2.0;
	
	public static ArrayList<Point2D.Double> getBoundaries(Point2D.Double center, double objectXSize, double objectYSize) {
      ArrayList<Point2D.Double> boundaries = new ArrayList<Point2D.Double>();
      
      boundaries.add(performOffset(new Point2D.Double(center.x + objectXSize/2, center.y - objectYSize/2)));
      boundaries.add(performOffset(new Point2D.Double(center.x + objectXSize/2, center.y + objectYSize/2)));
      boundaries.add(performOffset(new Point2D.Double(center.x - objectXSize/2, center.y + objectYSize/2)));
      boundaries.add(performOffset(new Point2D.Double(center.x - objectXSize/2, center.y - objectYSize/2)));
      
      return boundaries;
    }
    
    public static Point2D.Double performOffset(Point2D.Double pointToOffset){
        return new Point2D.Double(pointToOffset.x+(lengthOfMap/2), (widthOfMap/2)-pointToOffset.y);
    }

    private static double pointToLineDistance(double locX, double locY, double x1, double y1, double x2, double y2) {
        double a = locX - x1;
        double b = locY - y1;
        double c = x2 - x1;
        double d = y2 - y1;
        double e = -d;
        double f = c;
        
        double dot = a*e+b*f;
        double lenSquare = e*e + f*f;
        
        return Math.abs(dot)/Math.sqrt(lenSquare);
    }
}