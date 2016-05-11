# LibCityGML Utils

This library regroups several classes and methods useful for processes related to cityGML objects.

## Dependencies

- **LibCityGML**

## Functionalities

### Axis Aligned Bounding Boxes (AABB)

The files AABB.hpp and AABB.cpp have been designed to compute, save and compare the bounding boxes of cityGML objects.

- **Build AABB:**

To build the AABB of your cityGML objects stored in a directory ('dir'), you can use the following function :

	void BuildAABB(std::string dir);

The directory must be structured in subdirectories which will each represent a data layer and be associated with a suffix. As for now, there is four data layers:

| Layer | Suffix |
| -------- | -------- |
| Building | _BATI |
| Field | _MNT |
| Water | _WATER |
| Vegetation | _VEGET |


The cityGML files associated with each layer must be placed in the corresponding subdirectory.

*Note: The cityGML files must each represent one Tile.*

The BuildAABB function will read all the cityGML files and create for each layer a text file storing all the AABB. This text file will be named "<Suffix\>_AABB.dat". For example, for the building layer, the function will create a file named "_BATI_AABB.dat". 

The text files are structured as follow:

><Box number\>
><#AABB 1\>
><#AABB 2\>
>...
><#AABB N\>

And each #AABB is:

><Name of the box\>
><Minimum X\>
><Minimum Y\>
><Minimum Z\>
><Maximum X\>
><Maximum Y\>
><Maximum Z\>

Where Minimum and Maximum are the min and max points of the AABB.

*Note: The building must be done once and there is no need to redo it until you add/remove or change the cityGML files.*

- **Load AABB**

Once the text files have been created, they can be loaded using:

	 AABBCollection LoadAABB(std::string dir);

This method will load the AABB files in the 'dir' directory and return an AABBCollection.

An AABBCollection is a structure (defined in AABB.hpp) which holds a vector of AABB for each layer (Building, Field, Water, Vegetation, ..).

### Triangle

The files Triangle.hpp and Triangle.cpp holds Triangle and TriangleList structures and a method to build a list of triangles from a cityGML tile.

- **Triangle** holds information about a triangle (points, cityGML object type and Id, tileFile, ...). All the members of this structure are listed in the doxygen documentation.

- **TriangleList** holds a vector of Triangle*.

- **BuildTriangleList** method is meant to create a TriangleList from a cityGML tile file:

		TriangleList* BuildTriangleList(std::string tilefilename, citygml::CityObjectsType objectType);

Given the filename of a tile and the type of cityGML objects to load, it will return a pointer to a TriangleList.
