#include "criteria.h"
#include <cmath>

Criteria::Criteria()
{

}

double Criteria::getSpatialSatisfactionDegree(int currentLOD, int documentLOD)
{
    double satisfactionDegree = 1.0/(1.0 + std::fabs(currentLOD - documentLOD));
    return satisfactionDegree;
}
double Criteria::getTemporalSatisfactionDegree(time_t time1, time_t time2)
{
   double satisfactionDegree = 1.0/(1.0 + std::fabs(difftime(time1,time2)));
    return satisfactionDegree;
}
double Criteria::getProviderSatisfactionDegree()
{
    double satisfactionDegree = 1.0;
    return satisfactionDegree;
}
double Criteria::getThematicSatisfactionDegree()
{
    double satisfactionDegree = 1.0;
    return satisfactionDegree;
}
double Criteria::getContentSatisfactionDegree()
{
    double satisfactionDegree = 1.0;
    return satisfactionDegree;
}

double Criteria::getOverallSatisfactionDegree(Document d)
{
    double satisfactionDegree = 1.0;
    return satisfactionDegree;
}
