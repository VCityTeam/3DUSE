// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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
