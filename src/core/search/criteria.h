#ifndef CRITERA_H
#define CRITERA_H

#include "document.h"

class Criteria
{
private:
    double getSpatialSatisfactionDegree(int currentLOD, int documentLOD);
    double getTemporalSatisfactionDegree(time_t time1, time_t time2);
    double getProviderSatisfactionDegree();
    double getThematicSatisfactionDegree();
    double getContentSatisfactionDegree();

public:
    Criteria();
    double getOverallSatisfactionDegree(Document);
};

#endif // CRITERA_H
