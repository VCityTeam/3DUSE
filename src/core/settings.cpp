// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#include "settings.hpp"
#include <QSettings>
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
    ////////////////////////////////////////////////////////////////////////////////
    Settings::Settings()
        : m_loadTextures(true)
    {
        QSettings settings("liris", "virtualcity");
        m_loadTextures = settings.value("loadtextures").toBool();
        m_startDate = settings.value("startDate", "2000-01-01T00:00:00").toString().toStdString();
        m_endDate = settings.value("endDate", "2001-01-01T00:00:00").toString().toStdString();
        m_incSize = settings.value("incSize", 1).toInt();
        m_incIsDay = settings.value("incIsDay", true).toBool();
    }
    ////////////////////////////////////////////////////////////////////////////////
    DataProfile& Settings::getDataProfile()
    {
        return m_dataprofile;
    }
    ////////////////////////////////////////////////////////////////////////////////
    const DataProfile& Settings::getDataProfile() const
    {
        return m_dataprofile;
    }
    ////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
