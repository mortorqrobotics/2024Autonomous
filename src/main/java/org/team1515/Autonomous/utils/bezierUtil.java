package org.team1515.Autonomous.utils;

import java.util.ArrayList;
import java.util.function.DoubleFunction;

public class bezierUtil {

    public static ArrayList<Equation> derivativeEquation(ArrayList<Point> arr){
        ArrayList<Equation> result = new ArrayList<Equation>();
        int n = arr.size()-1;
        for (int i = 0; i <= n-1; i++){
            int integar=i;
            // System.out.print(integar);
            double x1 = arr.get(i).x;
            double y1 = arr.get(i).y;
            double x2 = arr.get(i+1).x;
            double y2 = arr.get(i+1).y;
            DoubleFunction<Double> term1 = (double t) -> binomialCo(n-1,integar);
            DoubleFunction<Double> term2 = (double t) -> Math.pow((1.0-t), (n-1)-integar);
            DoubleFunction<Double> term3 = (double t) -> Math.pow(t,integar);
            result.add(new Equation(
                ((double t) -> n*term1.apply(t)*term2.apply(t)*term3.apply(t)*(x2-x1)), 
                ((double t) -> n*term1.apply(t)*term2.apply(t)*term3.apply(t)*(y2-y1))
            ));
        }
        return result;
    }
    public static ArrayList<Equation> bezierEquation(ArrayList<Point> arr){
        ArrayList<Equation> result = new ArrayList<Equation>();
        int n = arr.size()-1;
        for (int i = 0; i <= n; i++){
            int integar=i;
            // System.out.print(integar);
            double x1 = arr.get(i).x;
            double y1 = arr.get(i).y;
            DoubleFunction<Double> term1 = (double t) -> binomialCo(n,integar);
            DoubleFunction<Double> term2 = (double t) -> Math.pow((1.0-t), n-integar);
            DoubleFunction<Double> term3 = (double t) -> Math.pow(t,integar);
            result.add(new Equation(
                ((double t) -> term1.apply(t)*term2.apply(t)*term3.apply(t)*x1), 
                ((double t) -> term1.apply(t)*term2.apply(t)*term3.apply(t)*y1)
            ));
        }
        return result;
    }

    public static double binomialCo(int n, int i){
        if (i == 0){
            return 1.0;
        }
        return factorial(n)/(factorial(i)*factorial(n-i));
    }

    public static long factorial(long n) {
        if (n <= 1)
            return 1;
        else
            return n * factorial(n - 1);
    }

    public static CartesianPoint[] placePoints(ArrayList<Equation> bezierEquation, int points){
        CartesianPoint[] placedPoints = new CartesianPoint[points];
        //loops from 0 to 1 incrementing by 1 over the number of sections(points-1)
        //for each section, calculates the point at said section and adds it to the new arraylist
        for(int i = 0; i <= 1; i+=(1.0/(points-1))){
            double x=0.0;
            double y=0.0;
            //calculates the values of x and y
            for(Equation e: bezierEquation){
                x+=e.applyX(i);
                y+=e.applyY(i);
            }
            placedPoints[i] = new CartesianPoint(x, y);
        }
        return placedPoints;
    }

    public static double getLength(CartesianPoint start, CartesianPoint end){
        return Math.sqrt(Math.pow((end.getX()-start.getX()), 2)+Math.pow((end.getY()-start.getY()), 2));
    }

    public static double bezierLength(CartesianPoint[] points){
        double total = 0.0;
        for(int i=0; i<points.length-1;i++){
            total+=getLength(points[i], points[i+1]);
        }
        return total;
    }

    public static CartesianPoint applyBezierEquation(ArrayList<Equation> bezierEquation, double t){
        double i = 0.0;
        double j = 0.0;
        for(Equation p : bezierEquation){
            i+=p.applyX(t);
            j+=p.applyY(t); 
        }
        return new CartesianPoint(i, j);
    }

    public static CartesianPoint[] spacedPoints(ArrayList<Equation> bezierEquation){
        //return value
        //number of points on our bezier apporximation
        int n = 10;
        //set of points on our bezier
        CartesianPoint[] points = placePoints(bezierEquation, n-2);
        //use helper functions to calculate average leg length
        double avgLegLength = bezierLength(points)/(n-1);
        //set how much we nudge each point every round to(honetsly idk)
        double stepSize = (1/n)/10;
        //create an array of t values
        double[] tValues = new double[n];
        for(int i = 0;i<n;i++){
            tValues[i] = (double)i/n;
        }
        //4 rounds of nudging
        for(int i = 0; i<4; i++){
            //list of lengths of each bezier section
            double[] legLengths = new double[n-1];
            for(int j = 0; j<n-1;j++){
                //note: leg[0] is the leg from point[0] to point[1]
                legLengths[j] = getLength(points[j], points[j+1]);
            }
            //difference between real leg length and average leg length
            double[]legError = new double[legLengths.length];
            for(int j = 0; j<legError.length;j++){
                legError[j] = legLengths[j]-avgLegLength;
            }
            //loops thrugh each t(not including first and last) value and nudges it left or right based on legError
            for(int j = 1; j<tValues.length-1;j++){
                //moves t in the opposite direction of the error by the step size times the error
                tValues[j] += (stepSize*legError[j-1]*-1.0);
                //sets the point to be at that t value
                points[j] = applyBezierEquation(bezierEquation, tValues[j]);
            }
        }
        return points;
    }
}   