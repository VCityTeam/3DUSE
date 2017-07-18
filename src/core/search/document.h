// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef DOCUMENT_H
#define DOCUMENT_H

#include <string>
#include <ctime>
#include <list>

class Document
{
    std::string name;
    std::string provider;
    // FIXME int lod_level;
    // FIXME std::time_t publicationDate;

public:
    Document();

};

#endif // DOCUMENT_H
