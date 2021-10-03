import java.util.ArrayList;
import java.awt.geom.Point2D;

/**
 * Test cases for 'polygonObject.java'.
 */
public class testPolygonObject {
    /**
     * Runs test cases.
     * @param args
     */
    public static void main(String[] args){
        //build starting object
        ArrayList<Point2D.Double> seed = new ArrayList<Point2D.Double>();
        seed.add(new Point2D.Double(9,9));
        seed.add(new Point2D.Double(9,8));
        seed.add(new Point2D.Double(8,8));
        seed.add(new Point2D.Double(8,9));
        Point2D.Double centroid = new Point2D.Double(8.5, 8.5);
        polygonObject test = new polygonObject(seed, centroid);
        //test object
        System.out.println("POLYGON BUILT");
        System.out.println("perimeter corner coordinates in order: (9, 9) -> (9, 8) -> (8, 8) -> (8, 9)");
        System.out.println("center position: (8.5, 8.5)");
        System.out.println("TESTING CONTAINS()...");
        System.out.println("Point on the an edge (9, 8.5)    - Expecting True, outcome=expected: " + (true==test.contains(new Point2D.Double(9,8.5))));
        System.out.println("Point on a corner (9, 8)         - Expecting True, outcome=expected: " + (true==test.contains(new Point2D.Double(9,8))));
        System.out.println("Point on the inside (8.4, 8.2)   - Expecting True, outcome=expected: " + (true==test.contains(new Point2D.Double(8.4,8.2))));
        System.out.println("Point on the inside (8.3, 8.5)   - Expecting True, outcome=expected: " + (true==test.contains(new Point2D.Double(8.3,8.5))));
        System.out.println("Point on the inside (8.5, 8.5)   - Expecting True, outcome=expected: " + (true==test.contains(new Point2D.Double(8.5,8.5))));
        System.out.println("Point on the outside (33, 3)    - Expecting False, outcome=expected: " + (false==test.contains(new Point2D.Double(33,3))));
        System.out.println("Point on the outside (-1, -3)   - Expecting False, outcome=expected: " + (false==test.contains(new Point2D.Double(-1,-3))));
        System.out.println("Point on the outside (0, 1)     - Expecting False, outcome=expected: " + (false==test.contains(new Point2D.Double(0,1))));
        System.out.println("Point on the outside (3, -62)   - Expecting False, outcome=expected: " + (false==test.contains(new Point2D.Double(3,-62))));
        System.out.println("Point on the outside (100, 200) - Expecting False, outcome=expected: " + (false==test.contains(new Point2D.Double(100,200))));
        System.out.println("TESTING ROTATE()...");
        //TODO - test rotate function
    }
}
