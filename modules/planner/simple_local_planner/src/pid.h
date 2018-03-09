#ifndef __PID__H
#define __PID__H

#include  <vector>
#include <algorithm>

using namespace std;

class Smoother
{
private:
    int size;
    vector<double> array;
public:
    Smoother():size(5)
    {clear();}

    void set_queue_size(int size)
    {
        if(size<0) size = 5;
        this->size = size;
    }

    void feed(double input)
    {
        if(array.size() < size)
        {
            array.push_back(input);
        }
        else
        {
            array.erase(array.begin());
            array.push_back(input);
        }
    }

    double run()
    {
        return get_mean();
    }

private:

    double get_mean(void)
    {
        double sum = std::accumulate(std::begin(array), std::end(array), 0.0);
        double mean =  sum / array.size(); //均值
        return mean;
    }

    double get_stdev(void)
    {
        double sum = std::accumulate(std::begin(array), std::end(array), 0.0);
        double mean =  sum / array.size(); //均值
        return mean;

        double accum  = 0.0;
        std::for_each (std::begin(array), std::end(array), [&](const double d) {
            accum  += (d-mean)*(d-mean);
        });

        double stdev = sqrt(accum/(array.size()-1)); //方差
        return stdev;
    }

    void clear()
    {
        array.clear();
    }
};

class PID
{
private:
    double Kp;
    double Ki;
    double Kd;
    double sum;

    double err;//变量：当前的偏差e(k)
    double err_last;//历史：前一步的偏差e(k-1)
    double intergral;
    double intergral_max;

    double out_max;//参数：PID控制器的最大输出
    double out_min;//参数：PID控制器的最小输出
    Smoother smoother;

public:
    PID()
    {}

    void init(double kp,double ki,double kd, int smother_len)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;

        err = 0;
        sum = 0;
        err_last = 0;
        intergral = 0;

        intergral_max = 0.5;
        out_max = 0.4;
        out_min = -0.4;

        smoother.set_queue_size(smother_len);
    }

    double calc(double setPoint, double feedBack)
    {
        err = setPoint - feedBack;

        intergral += err;
        if(intergral >=  intergral_max)  intergral =  intergral_max;
        if(intergral <= -intergral_max) intergral = -intergral_max;
        sum = Kp*err + Ki*intergral + Kd*(err - err_last);
        err_last = err;

        if (sum > out_max)
            sum = out_max;
        if (sum < out_min)
            sum = out_min;

        smoother.feed(sum);
        return smoother.run();
    }

    void set_param(double kp, double ki, double kd)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    void set_max_min_output(double max, double min)
    {
        out_min = min;
        out_max = max;
    }

    void set_max_intergral(double max)
    {
        intergral_max = fabs(max);
    }

};



#endif
