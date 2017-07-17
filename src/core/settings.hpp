// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef __SETTINGS_HPP__
#define __SETTINGS_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "dataprofile.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
    ////////////////////////////////////////////////////////////////////////////////
    /// \brief The Settings class
    ///
    /// Store application parameters
    class Settings
    {
    public:
        Settings();

        /// \brief getDataProfile Get the data profile
        /// \return The data profile
        DataProfile& getDataProfile();

        /// \brief getDataProfile Get the data profile
        /// \return The data profile
        const DataProfile& getDataProfile() const;

        bool m_loadTextures;                ///< flag to load texture or not (in osg)
        DataProfile m_dataprofile;          ///< data profile

        std::string m_startDate;
        std::string m_endDate;
        int m_incSize;
        bool m_incIsDay;

    private:
    };
    ////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __SETTINGS_HPP__
