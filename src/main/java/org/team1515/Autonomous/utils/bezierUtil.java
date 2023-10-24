package org.team1515.Autonomous.utils;

import java.util.ArrayList;
import java.util.function.DoubleFunction;

import edu.wpi.first.math.Pair;

public class bezierUtil {

    public static ArrayList<Pair<Double, Double>> derivativeBezier(ArrayList<Pair<Double, Double>> arr){
        ArrayList<Pair<Double, Double>> result = new ArrayList<Pair<Double, Double>>();
        int n = arr.size();
        for (int i = 0; i < n-1; i++){
            result.add(new Pair<Double, Double>(n * (arr.get(i).getFirst() - arr.get(i+1).getFirst()), n * (arr.get(i).getSecond() - arr.get(i+1).getSecond())));
        }
        return result;
    }
    public static ArrayList<Pair<DoubleFunction<Double>, DoubleFunction<Double>>> bezierEquation(ArrayList<Pair<Double, Double>> arr){
        ArrayList<Pair<DoubleFunction<Double>, DoubleFunction<Double>>> result = new ArrayList<Pair<DoubleFunction<Double>, DoubleFunction<Double>>>();
        int n = arr.size();
        for (int i = 0; i < n; i++){
            final int integar=i;
            double x = arr.get(i).getFirst();
            double y = arr.get(i).getSecond();
            result.add(new Pair<DoubleFunction<Double>, DoubleFunction<Double>>(
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