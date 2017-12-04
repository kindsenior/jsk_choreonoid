/**
   @author Kunio Kojima
*/
#pragma once

#include <vector>
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

}
