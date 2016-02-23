#include <ros_cpg_wrapper/cpg_control.h>

using namespace cpg_control;

CpgController::CpgController(const int64_t& numIterations): m_numIterations(numIterations),
m_cpgParam0(0.0), m_cpgParam1(0.0), m_cpgParam2(0.0)
{
  
}

std_msgs::Int64 CpgController::GetNumIterations()
{
  std_msgs::Int64 numIterations;
  numIterations.data = m_numIterations;
  return numIterations;
}

void CpgController::SetNumIterations(const std_msgs::Int64& msg)
{
  m_numIterations = msg.data;
}

void CpgController::SetCpgParameter0(const std_msgs::Float64& msg)
{
  m_cpgParam0 = msg.data;
}

void CpgController::SetCpgParameter1(const std_msgs::Float64& msg)
{
  m_cpgParam1 = msg.data;
}

void CpgController::SetCpgParameter2(const std_msgs::Float64& msg)
{
  m_cpgParam2 = msg.data;
}

bool CpgController::GetHasNewIteration() const
{
  return m_hasNewIteration;
}

void CpgController::SetHasNewIteration(const bool& value)
{
  m_hasNewIteration = value;
}