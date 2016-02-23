#ifndef CPG_CONTROL_H
#define CPG_CONTROL_H

#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

namespace cpg_control
{
  class CpgController
  {
    public:
      CpgController(const int64_t& numIterations = 300);
      std_msgs::Int64 GetNumIterations();
      void SetNumIterations(const std_msgs::Int64& iterations);
      
      void SetCpgParameter0(const std_msgs::Float64& param);
      void SetCpgParameter1(const std_msgs::Float64& param);
      void SetCpgParameter2(const std_msgs::Float64& param);
      
      std_msgs::Float64 GetCpgParameter0();
      std_msgs::Float64 GetCpgParameter1();
      std_msgs::Float64 GetCpgParameter2();
      
      bool GetHasNewIteration() const;
      void SetHasNewIteration(const bool&);
      
    private:
      int64_t m_numIterations;
      double m_cpgParam0;
      double m_cpgParam1;
      double m_cpgParam2;
      
      bool m_hasNewIteration;
  };
} // namespace cpg_control

#endif // CPG_CONTROL_H