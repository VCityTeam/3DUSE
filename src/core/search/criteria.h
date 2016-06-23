#ifndef CRITERA_H
#define CRITERA_H

#include "document.h"

class Criteria
{
private:
    float getSpatialSatisfactionDegree(int currentLOD, int documentLOD);
    float getTemporalSatisfactionDegree(time_t time1, time_t time2);
    float getProviderSatisfactionDegree();
    float getThematicSatisfactionDegree();
    float getContentSatisfactionDegree();

public:
    Criteria();
    float getOverallSatisfactionDegree(Document);
};

#endif // CRITERA_H
