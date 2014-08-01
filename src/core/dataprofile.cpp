////////////////////////////////////////////////////////////////////////////////
#include "dataprofile.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
DataProfile::DataProfile()
    : m_bboxLowerBound(), m_bboxUpperBound(), m_offset(), m_xStep(), m_yStep()
{

}
////////////////////////////////////////////////////////////////////////////////
DataProfile createDataProfileDefault()
{	
	return createDataProfileLyon();

	DataProfile dp;
    dp.m_offset.x = 0.0;
    dp.m_offset.y = 0.0;

    dp.m_xStep = 0;
    dp.m_yStep = 0;

    dp.m_id = 0;
    dp.m_name = "None";

    return dp;
}
////////////////////////////////////////////////////////////////////////////////
DataProfile createDataProfileParis()
{
    DataProfile dp;
    dp.m_offset.x = 643000.0;
    dp.m_offset.y = 6857000.0;

    dp.m_bboxLowerBound.x = 643000.0;
    dp.m_bboxLowerBound.y = 6857000.0;

    dp.m_bboxUpperBound.x = 658000.0;
    dp.m_bboxUpperBound.y = 6867500.0;

    dp.m_xStep = 500;
    dp.m_yStep = 500;

    dp.m_TileIdOriginX = 1286;
    dp.m_TileIdOriginY = 13714;

    dp.m_id = 1;
    dp.m_name = "Paris";

    return dp;
}
////////////////////////////////////////////////////////////////////////////////
DataProfile createDataProfileLyon()
{
    DataProfile dp;
    dp.m_offset.x = 1842500.0;//   1840000.0 		 1842500.0
    dp.m_offset.y = 5175800.0;//   5170000.0			 5175800.0

    dp.m_bboxLowerBound.x = 1842500.0;
    dp.m_bboxLowerBound.y = 5172500.0;

    dp.m_bboxUpperBound.x = 1848000.0;
    dp.m_bboxUpperBound.y = 5175500.0;

    dp.m_xStep = 500;
    dp.m_yStep = 500;

    dp.m_TileIdOriginX = 0;
    dp.m_TileIdOriginY = 0;

    dp.m_id = 2;
    dp.m_name = "Lyon";

    return dp;
}
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
