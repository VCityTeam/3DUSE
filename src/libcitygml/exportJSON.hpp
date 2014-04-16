#ifndef __JSON_EXPORT_HPP_
#define __JSON_EXPORT_HPP_
////////////////////////////////////////////////////////////////////////////////
#include <QDateTime>
#include "citygml.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The ExporterJSON class
/// Export JSON
class ExporterJSON
{
public:
    ExporterJSON();

    void genConfigFiles();

    /// \brief exportCityModel
    /// \param model
    /// \param fileName
    void exportCityModel(CityModel* model, const std::string& fileName);

    /// \brief exportCityObject
    /// \param model
    /// \param fileName
    void exportCityObject(CityObject* model, const std::string& fileName);

    /// Enable or disable temporal export
    void setTemporalExport(bool param);

    /// Set temporal export date
    void setDate(const QDateTime& date);

private:
    bool m_temporalExport;  ///< enable temporal export
    QDateTime m_date;       ///< date for temporal export
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __JSON_EXPORT_HPP_
