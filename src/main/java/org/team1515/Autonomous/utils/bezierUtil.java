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


}