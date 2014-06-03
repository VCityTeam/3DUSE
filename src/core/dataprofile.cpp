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
	DataProfile dp;
    dp.m_offset.x = 0.0;
    dp.m_offset.y = 0.0;

    dp.m_xStep = 0;
    dp.m_yStep = 0;

    dp.m_id = 0;
    dp.m_name = "None";

    return dp;
	//return createDataProfileLyon();
}
////////////////////////////////////////////////////////////////////////////////
DataProfile createDataProfileParis()
{
    DataProfile dp;
    dp.m_offset.x = 643000.0;
    dp.m_offset.y = 6857000.0;

    dp.m_xStep = 500;
    dp.m_yStep = 500;

    dp.m_id = 1;
    dp.m_name = "Paris";

    return dp;
}
////////////////////////////////////////////////////////////////////////////////
DataProfile createDataProfileLyon()
{
    DataProfile dp;
    dp.m_offset.x = 1840000.0;
    dp.m_offset.y = 5170000.0;

    dp.m_xStep = 0;
    dp.m_yStep = 0;

    dp.m_id = 2;
    dp.m_name = "Lyon";

    return dp;
}
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
