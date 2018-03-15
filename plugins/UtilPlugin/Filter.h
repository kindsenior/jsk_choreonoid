/**
   @author Kunio Kojima
*/
#pragma once

#include <vector>
#include <deque>
#include <algorithm>
#include <iostream>

#include <cnoid/src/Util/EigenTypes.h>

namespace cnoid {

class MedianFilter{
protected:
    std::vector< std::deque<double> > dequeVec;
    int dataDim;
    int filterLength;

public:
    MedianFilter(int _dataDim, int _filterLength = 3){
        dataDim = _dataDim;
        filterLength = _filterLength;
        if(filterLength < 3) filterLength = 3;
        dequeVec.resize(dataDim);
    }

    VectorXd update(const VectorXd& newData){
        VectorXd retData = newData;
        for(int i=0; i<dataDim; ++i){
            std::deque<double> dataDeque = dequeVec[i];
            while(dataDeque.size() < filterLength+1){
                dataDeque.push_back(newData(i));
            }
            dataDeque.pop_front();
            dequeVec[i] = dataDeque;
            std::sort(dataDeque.begin(), dataDeque.end());

            retData(i) = dataDeque[filterLength/2];
        }
        return retData;
    }
};

template <class T> class MovingAverageFilter
{
protected:
    std::deque<T> dataDeque_;
    int filterLength_;

public:
    MovingAverageFilter(const int filterLength, const T init_value) {
        filterLength_ = filterLength;
        if(filterLength_ < 1) filterLength_ = 1;
        while(dataDeque_.size() < filterLength_) dataDeque_.push_back(init_value);
        // std::cout << "length:" << filterLength_ << std::endl;
        // for(auto data : dataDeque_) std::cout << " " << data(2); std::cout << std::endl;
    }

    T passFilter(T input) {
        T retData = input - input;
        // for(auto data : dataDeque_) std::cout << " " << data(2); std::cout << std::endl;
        dataDeque_.pop_front();
        dataDeque_.push_back(input);
        // for(auto data : dataDeque_) std::cout << " " << data(2); std::cout << std::endl;
        for(auto data : dataDeque_) retData += data;
        retData /= filterLength_;
        // std::cout << "ret " << retData.transpose() << std::endl;
        return retData;
    }
};

template <class T> class FirstOrderLowPassFilter
{
private:
    T prev_value;
    double cutoff_freq, dt, const_param;
public:
    FirstOrderLowPassFilter (const double _cutoff_freq, const double _dt, const T init_value) : prev_value(init_value), dt(_dt)
    {
        setCutOffFreq(_cutoff_freq);
    };
    ~FirstOrderLowPassFilter()
    {
    };
    T passFilter (T value)
    {
        prev_value = 1.0/(1+const_param) * prev_value + const_param/(1+const_param) * value;
        return prev_value;
    };
    void reset (T value) { prev_value = value; };
    void setCutOffFreq (const double f)
    {
        cutoff_freq = f;
        const_param = 2 * M_PI * cutoff_freq * dt;
    };
    double getCutOffFreq () const { return cutoff_freq; };
    T getCurrentValue () const { return prev_value; };
};

}
