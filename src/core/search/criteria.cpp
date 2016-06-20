#include "criteria.h"
#include <cmath>

Criteria::Criteria()
{

}

float Criteria::getSpatialSatisfactionDegree(int currentLOD, int documentLOD)
{
    return 1.0/(1.0 + std::abs(currentLOD - documentLOD));
}
float Criteria::getTemporalSatisfactionDegree(time_t time1, time_t time2)
{
    return 1.0/(1.0 + std::abs(difftime(time1,time2)));
}
float Criteria::getProviderSatisfactionDegree()
{

}
float Criteria::getThematicSatisfactionDegree()
{

}
float Criteria::getContentSatisfactionDegree()
{

}

float Criteria::getOverallSatisfactionDegree(Document d)
{

}
