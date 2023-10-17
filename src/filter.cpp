#include "ros/ros.h"
#include "filter.h"

/**
 * @brief medianFilter 中值滤波器
 * @param data 输入数据
 * @param size 输入数据的元素数量
 */
float medianFilter(float input[], int size)
{
    float temp;
    float data[size];
    int num = size;
    for(int i=0;i<size;i++)
        data[i] = input[i];
        
    while(size > 1)
    {
        for(int i = 0; i < size - 1; i++)
        {
            if(data[i] > data[i + 1])
            {
                temp = data[i];
                data[i] = data[i + 1];
                data[i + 1] = temp;
            }
        }
        size --;
    }
    return data[num/2];
}