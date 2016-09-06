#ifndef LP_FILTER_H
#define LP_FILTER_H

#include <vector>
#include <ros/ros.h>
#include <math.h>

class LPFilter
{
public:
  LPFilter(double deltaT, double cutoffFrequency, int numElements);
  bool update(std::vector<double> input, std::vector<double>& output);

private:
  bool initialized;
  int noElements;
  std::vector<double> in1, in2, out1, out2;
  double omega_a;
  double a0, a1, a2, b1, b2;
};

#endif