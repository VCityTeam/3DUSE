See also the [reflexions on libCityGML next generation](https://github.com/MEPP-team/VCity/wiki/libCityGML_NG) and [Xerces vs libXML2 issue](https://github.com/MEPP-team/VCity/issues/164)

## Dependencies: to be asserted

| Package         |    License    | Included headers / Notes |
| --------------- | ------------- | ------------------------ |
| [libXml2](http://www.xmlsoft.org/) | [MIT licence](http://www.xmlsoft.org/) ||
|[ASSIMP](http://assimp.sourceforge.net/main_doc.html) | [BSD](http://assimp.sourceforge.net/main_license.html)|[OSGPL](http://trac.openscenegraph.org/projects/osg//wiki/Legal) an LGPL variant|
|[OpenSceneGraph (OSG)](http://www.openscenegraph.org/)||Sub-libraries: osgDB |

**IMPORTANT NOTE: [citymodel.cpp](https://github.com/MEPP-team/VCity/blob/master/src/libcitygml/citymodel.cpp) very suspiciously includes `gui/applicationGui.hpp` which annouces some unwanted QT dependency !!!**

## General notes
 * No written use cases for the current development work focused on a scenario extension (blended or "on top" of the temporal extension). 
 * Since they are no use cases, they are no requirements nor design notes. Current coding work is thus totally informal although some major structural difficulties are foreseen e.g. :
    * How will the Temporal ADE (Application Domain Extension) be articulated with the upcoming scenarios ADE ? How to compose ADEs ? How is this forwarded to the manually crafted implementation (see below for the alignment issue) ?
    * How does the derivative Temporal ADE (Application Domain Extension) work blend with [other ADE](http://www.citygmlwiki.org/index.php/CityGML-ADEs) not conceived at LIRIS. Is this an "in silo" effort (i.e. not related with the rest of the CityGLM community) ? 
    * More generally how can a development be realized without use cases at hand (this is worth than shooting a moving target, is it shooting with no target) ?
 * There is a dependency of the [temporel plugin](https://github.com/MEPP-team/VCity/tree/master/doc/Temporel/) towards both [Enterprise Architect](http://www.sparxsystems.eu/) (which is Windoze only and commercial) and [ShapeChange](http://shapechange.net/) (java32bits VM only) for the XSD part (which seems to be acting as definition of this Application Domain Extension) at least to change, modify, fix the generated XSD.
 * The classes of the implementation are generated with [SAX](http://sax.sourceforge.net/) macros (refer to [parser.cpp](https://github.com/MEPP-team/VCity/blob/master/src/libcitygml/parser.cpp)). As a consequence: 
   * the attributes are loaded on the fly (in a map) as text. 
   * There is no “real” (thorough) validation of the XML file and what is asserted is the existence of the attributes.
   * There is no construction of a class per CityGML object but a generic data structure that gets constructed dynamically (on import) and where the attributes are represented as strings… 
   * Such implementation is heavy in memory (think of multi-gig files for New-York), error prone (I can put a string with a year “ten thousand” instead of the number 10000 and the loading won’t break…) and hard to document (doxygen won’t generate anything meaningful when working on macros).
* The manually crafted C++ data bindings (libCityGML) and the associated XSD defined concrete form are **manually aligned** (which the developers' brain as only tool as opposed to guaranteed by some software tools e.g. Enterprise Architect can generate code).
* Within [importerAssimp.cpp](https://github.com/MEPP-team/VCity/blob/master/src/libcitygml/import/importerAssimp.cpp) some large portions of commented out code should be cleaned up.
