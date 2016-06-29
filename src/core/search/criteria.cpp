#include "criteria.h"
#include <cmath>

Criteria::Criteria()
{

}

float Criteria::getSpatialSatisfactionDegree(int currentLOD, int documentLOD)
{
    return 1.0/(1.0 + std::fabs(currentLOD - documentLOD));
}
float Criteria::getTemporalSatisfactionDegree(time_t time1, time_t time2)
{
    return 1.0/(1.0 + std::fabs(difftime(time1,time2)));
}
float Criteria::getProviderSatisfactionDegree()
{
    float satisfactionDegree = 1.0;
    return satisfactionDegree;
}
float Criteria::getThematicSatisfactionDegree()
{
    float satisfactionDegree = 1.0;
    return satisfactionDegree;
}
float Criteria::getContentSatisfactionDegree()
{
    float satisfactionDegree = 1.0;
    return satisfactionDegree;
}

float Criteria::getOverallSatisfactionDegree(Document d)
{
    float satisfactionDegree = 1.0;
    return satisfactionDegree;
}
