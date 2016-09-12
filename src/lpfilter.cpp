/*
Copyright (c) 2016, Los Alamos National Security, LLC
All rights reserved.
Copyright 2016. Los Alamos National Security, LLC. This software was produced under U.S. Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S. Government has rights to use, reproduce, and distribute this software.  NEITHER THE GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce derivative works, such modified software should be clearly marked, so as not to confuse it with the version available from LANL.

Additionally, redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 
3. Neither the name of Los Alamos National Security, LLC, Los Alamos National Laboratory, LANL, the U.S. Government, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Author: Alex von Sternberg
*/

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
