package org.team1515.Autonomous.utils;

import java.util.ArrayList;

public class bezierUtil {

    public static ArrayList<Point> derivativeBezier(ArrayList<Point> arr){
        ArrayList<Point> result = new ArrayList<Point>();
        int n = arr.size();
        for (int i = 0; i < n-1; i++){
            result.add(new Point(n * (arr.get(i).x - arr.get(i+1).x), n * (arr.get(i).y - arr.get(i+1).y)));
        }
        return result;
    }
    public static ArrayList<Equation> bezierEquation(ArrayList<Point> arr){
        ArrayList<Equation> result = new ArrayList<Equation>();
        int n = arr.size();
        for (int i = 0; i < n; i++){
            final int integar=i;
            double x = arr.get(i).x;
            double y = arr.get(i).y;
            result.add(new Equation(
                ((double t) -> binomialCo(n,integar)*Math.pow((integar-t), n-integar)*Math.pow(t,integar)*x), 
                ((double t) -> binomialCo(n,integar)*Math.pow((integar-t), n-integar)*Math.pow(t,integar)*y)
            ));
        }
        return result;
    }

    public static double binomialCo(int n, int i){
        if (i == 0){
            return 1.0;
        }
        return factorial(n)/(factorial(i)*factorial(n-1));
    }

    public static long factorial(long n) {
        if (n <= 1)
            return 1;
        else
            return n * factorial(n - 1);
    }


}