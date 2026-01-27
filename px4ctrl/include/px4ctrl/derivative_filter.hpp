#pragma once

#include <iostream>
#include <cmath>

/// @brief	A class to implement a derivative (slope) filter
/// See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/

template<class T, size_t FILTER_SIZE>
class DerivativeFilter
{
    private:
        bool     has_new_data = false;
        size_t   sample_index = 0;
        T        last_slope = 0;
        T        samples[FILTER_SIZE] = {0};
        double time_stamps[FILTER_SIZE] = {0};

        T add(T sample)
        {
            samples[sample_index++] = sample;

            if( sample_index >= FILTER_SIZE )
                sample_index = 0;

            return sample;
        }

    public:
        DerivativeFilter() {}
        ~DerivativeFilter() {}        

        void update(T sample, const double& time_stamp)
        {
            size_t i = sample_index;
            size_t i1;

            if (i == 0)
                i1 = FILTER_SIZE-1;
            else
                i1 = i-1;
 
            if (time_stamps[i1] == time_stamp)
                return;

            time_stamps[i] = time_stamp;

            add(sample);

            has_new_data = true;
        }

        inline T f(size_t i)
        {
            return samples[(((sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE];
        }

        inline double x(size_t i)
        {
            return time_stamps[(((sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE];
        }

        float slope()
        {
            if (!has_new_data)
                return last_slope;

            float result = 0.0f;

// use f() to make the code match the maths a bit better.
// Note that unlike an average filter, we care about the order of the elements
// #define f(i) samples[(((sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE]
// #define x(i) time_stamps[(((sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE]

            if (time_stamps[FILTER_SIZE-1] == time_stamps[FILTER_SIZE-2]) 
                return 0.0f;

            // N in the paper is FILTER_SIZE
            switch (FILTER_SIZE) 
            {
                case 5:
                    result = 2*2*(f(1) - f(-1)) / (x(1) - x(-1))
                            + 4*1*(f(2) - f(-2)) / (x(2) - x(-2));
                    result /= 8;
                    break;
                case 7:
                    result = 2*5*(f(1) - f(-1)) / (x(1) - x(-1))
                            + 4*4*(f(2) - f(-2)) / (x(2) - x(-2))
                            + 6*1*(f(3) - f(-3)) / (x(3) - x(-3));
                    result /= 32;
                    break;
                case 9:
                    result = 2*14*(f(1) - f(-1)) / (x(1) - x(-1))
                            + 4*14*(f(2) - f(-2)) / (x(2) - x(-2))
                            + 6* 6*(f(3) - f(-3)) / (x(3) - x(-3))
                            + 8* 1*(f(4) - f(-4)) / (x(4) - x(-4));
                    result /= 128;
                    break;
                case 11:
                    result =  2*42*(f(1) - f(-1)) / (x(1) - x(-1))
                            +  4*48*(f(2) - f(-2)) / (x(2) - x(-2))
                            +  6*27*(f(3) - f(-3)) / (x(3) - x(-3))
                            +  8* 8*(f(4) - f(-4)) / (x(4) - x(-4))
                            + 10* 1*(f(5) - f(-5)) / (x(5) - x(-5));
                    result /= 512;
                    break;
                default:
                    result = 0.0f;
                    break;
            }

            if (isnan(result) || isinf(result))
                result = 0.0f;

            has_new_data = false;
            last_slope = result;

            return result;
        }
};

typedef DerivativeFilter<double, 5> DerivFilter5;
typedef DerivativeFilter<double, 7> DerivFilter7;
typedef DerivativeFilter<double, 9> DerivFilter9;
typedef DerivativeFilter<double, 11> DerivFilter11;
