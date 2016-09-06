#include <lpfilter.h>

LPFilter::LPFilter(double deltaT, double cutoffFrequency, int numElements):
  initialized(false),
  noElements(0),
  omega_a(0.0),
  a0(0.0),
  a1(0.0),
  a2(0.0),
  b1(0.0),
  b2(0.0)
{
  initialized = true;
  if(numElements<=0)
  {
    ROS_ERROR_STREAM("LPFilter was passed invalid number of elements. Not filtering.");
    initialized = false;
  }
  else
  {
    noElements = numElements;
  }
  if(deltaT<=0)
  {
    ROS_ERROR_STREAM("LPFilter was passed invalid deltaT. Not Filtering.");
    initialized = false;
  }
  if(cutoffFrequency <=0)
  {
    ROS_ERROR_STREAM("LPFilter was passed invalid cuttoffFrequency. Not Filtering.");
    initialized = false;
  }
  else
  {
    cutoffFrequency *= (2*M_PI);
    omega_a = tan(cutoffFrequency*deltaT/2.0);
    double den = omega_a*omega_a+sqrt(2.0*omega_a+1.0);
    a0 = (omega_a*omega_a)/den;
    a1 = (2.0*omega_a*omega_a)/den;
    a2 = a0;
    b1 = 2.0*(omega_a*omega_a-1.0)/den;
    b2 = (omega_a*omega_a-sqrt(2.0)*omega_a+1)/den;
    in1.resize(noElements);
    in2.resize(noElements);
    out1.resize(noElements);
    out2.resize(noElements);
    ROS_INFO_STREAM("cutoffFrequency: " << cutoffFrequency << ". omega_a: " << omega_a << ". den: " << den << ". a0: " << a0 << ". a1: " << a1 << ". a2: " << a2 << ". b1: " << b1 << ". b2: " << b2);
  }
}
  
bool LPFilter::update(std::vector<double> input, std::vector<double>& output)
{
  if(!initialized)
  {
    ROS_ERROR_STREAM("LPFilter was not initialized correctly. Not filtering data.");
    return false;
  }
  if(input.size() != in1.size() || output.size() != out1.size())
  {
    ROS_ERROR_STREAM("LPFilter incorrect input or output size");
    return false;
  }
  for(int i=0; i<noElements; i++)
  {
    output.at(i) = a0*input.at(i) + a1*in1.at(i) + a2*in2.at(i) - b1*out1.at(i) - b2*out2.at(i);
    out2.at(i) = out1.at(i);
    out1.at(i) = output.at(i);
    in2.at(i) = in1.at(i);
    in1.at(i) = input.at(i);
  }
  return true;
} 