#ifndef DOCUMENT_H
#define DOCUMENT_H

#include <string>
#include <ctime>
#include <list>

class Document
{
    std::string name;
    std::string provider;
    int lod_level;
    std::time_t publicationDate;

public:
    Document();

};

#endif // DOCUMENT_H
