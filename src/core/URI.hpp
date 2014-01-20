#ifndef __URI_HPP__
#define __URI_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <vector>
#include <string>
////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////
/// \brief The URI class identifies a node
///
class URI
{
public:
    URI();

    /// \brief append Append a node name to the URI
    /// \param node The node name to append
    void append(const std::string& node);

    /// \brief setType Set the type of the final node
    /// \param type Teh type of the final node
    void setType(const std::string& type);

    /// \brief getDepth Get the number of depth level
    /// \return The number of depth level
    int getDepth() const;

    /// \brief getNode Get a node name given a depth level
    /// \param depth Depth level, starting from 0
    /// \return std::string with the name of the node at depth depth
    const std::string& getNode(int depth) const;

    /// \brief getType Get the type of the final node pointed by the URI
    /// \return The type of the final node pointed by the URI as a string
    const std::string& getType() const;

    /// \brief getURI Get the URI in a string array form
    /// \return The URI in a string array form
    std::vector<std::string>& getURI();

    /// \brief getURI Get the URI in a string array form
    /// \return The URI in a string array form
    const std::vector<std::string>& getURI() const;

    /// \brief getStringURI Convert the URI in a single string
    /// '.' is the separator
    /// \param includeType If true, type will be prepend to the string
    /// \return The URI converted into a string
    std::string getStringURI(bool includeType = true) const;

private:
    int m_depth;                    ///< Depth of the uri
    std::string m_type;             ///< Pointed node type
    std::vector<std::string> m_uri; ///< String array. One string per node name, one per depth level.
};
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////
#endif // __URI_HPP__
