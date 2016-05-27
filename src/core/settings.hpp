// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
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
