// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __LOG_HPP__
#define __LOG_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <string>
//#include <fstream> // MT 31/01/2014
#include <osgDB/fstream>
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
/// \brief Log helper class
/// allow to log using operator<<
class Log
{
public:
    Log();

    void setLogToFile(bool param);
    void setLogToStdout(bool param);

    Log& operator<<(const char* str);
    Log& operator<<(const std::string& str);
    Log& operator<<(int val);
    Log& operator<<(double val);

private:
    bool m_logFile;         ///< enable file logging ?
    bool m_logStdout;       ///< enable stdout logging ?
    /*std*/osgDB::ofstream m_file;  ///< file for logging  // MT 31/01/2014
};
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __LOG_HPP__
