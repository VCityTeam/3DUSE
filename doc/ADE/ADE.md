# ADE (Application Domain Extension)
CityGML can be extended in two ways
- Generic City objects and attributes
- Application Domain Extension (ADE)

Generic city objects and attributes can be used to extend CityGML during runtime whereas ADE can be used to extend CityGML in a manner that permits not only to add new feature types and new attributes to existing CityGML classes but also supports sharing this extension.

# Definition: ADE
ADE is used to extend CityGML for specifiying various application specific information.
These extensions can be in the form of specifying new properties to the existing CityGML classes or defining new object types.
An ADE has to be defined in an extra XSD (XML Schema Definition) file with its own namespace.
The extended CityGML instance documents can be validated against CityGML and respective ADE schemas.
Examples (given in the CityGML Official documents):
- ADE for noise emission simulation
- ADE for ubiquituous networks robots service

## Creation of ADE
Following extensions are possible through ADE.
- New feature types based on the abstract or concrete City GML classes are defined.
  Examples include deriving new feature types from abstract classes like \_CityObject or concrete classes like CityFurniture. 
- Existing feature types are extended by application specific properties. 
  These properties may have simple or complex data types. 

## Programming an ADE
- Import ADE.hpp
- Create a new ADE class inheriting ADEHandler and implement the functions startElement, endElement, endDocument

## Example ADE
- [Document ADE](DocumentADE/documentADE.md)
- [Temporal ADE](Temporel/temporalADE.md)
