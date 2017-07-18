// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef _DOCUMENTHANDLER_HPP_
#define _DOCUMENTHANDLER_HPP_

#include "../ADE.hpp"
#include "documentObject.hpp"
#include "reference.hpp"
#include "tag.hpp"

class DocumentHandler : public ADEHandler
{
protected :
    documentADE::DocumentObject* _currentDocument;
    documentADE::Reference* _currentReference;
    documentADE::Tag* _currentTag;
    std::vector<documentADE::DocumentObject*> _documents;
    std::vector<documentADE::Reference*> _references;
private:
    void setAttributeValue(std::string name);
    void setDocumentAttributeValue(std::string name);

public:
    DocumentHandler(void);
    DocumentHandler(citygml::CityGMLHandler* gmlHandler);

    std::string getAttribute( void* attributes, const std::string& attname, const std::string& defvalue );
    std::string removeNamespace(std::string name);
    std::string getIDfromQuery(std::string query);

    void startElement(std::string name, void* attributes);
    void endElement(std::string);
    void endDocument();
};

#endif
