// COTD Entry submitted by John W. Ratcliff [jratcliff@verant.com]

// ** THIS IS A CODE SNIPPET WHICH WILL EFFICIEINTLY TRIANGULATE ANY
// ** POLYGON/CONTOUR (without holes) AS A STATIC CLASS.  THIS SNIPPET
// ** IS COMPRISED OF 3 FILES, TRIANGULATE.H, THE HEADER FILE FOR THE
// ** TRIANGULATE BASE CLASS, TRIANGULATE.CPP, THE IMPLEMENTATION OF
// ** THE TRIANGULATE BASE CLASS, AND TEST.CPP, A SMALL TEST PROGRAM
// ** DEMONSTRATING THE USAGE OF THE TRIANGULATOR.  THE TRIANGULATE
// ** BASE CLASS ALSO PROVIDES TWO USEFUL HELPER METHODS, ONE WHICH
// ** COMPUTES THE AREA OF A POLYGON, AND ANOTHER WHICH DOES AN EFFICENT
// ** POINT IN A TRIANGLE TEST.
// ** SUBMITTED BY JOHN W. RATCLIFF (jratcliff@verant.com) July 22, 2000

#ifndef TRIANGULATE_H

#define TRIANGULATE_H

/*****************************************************************/
/** Static class to triangulate any contour/polygon efficiently **/
/** You should replace Vector2d with whatever your own Vector   **/
/** class might be.  Does not support polygons with holes.      **/
/** Uses STL vectors to represent a dynamic array of vertices.  **/
/** This code snippet was submitted to FlipCode.com by          **/
/** John W. Ratcliff (jratcliff@verant.com) on July 22, 2000    **/
/** I did not write the original code/algorithm for this        **/
/** this triangulator, in fact, I can't even remember where I   **/
/** found it in the first place.  However, I did rework it into **/
/** the following black-box static class so you can make easy   **/
/** use of it in your own code.  Simply replace Vector2d with   **/
/** whatever your own Vector implementation might be.           **/
/*****************************************************************/


#include <vector>  // Include STL vector class.

#include "vecs.hpp" // MT
class Vector3d // MT
{
public:
  Vector3d(double x,double y,double z)
  {
    Set(x,y,z);
  };

  double GetX(void) const { return mX; };

  double GetY(void) const { return mY; };

  double GetZ(void) const { return mZ; };

  void Set(double x,double y,double z)
  {
    mX = x;
    mY = y;
	mZ = z;
  };
private:
  double mX;
  double mY;
  double mZ;
};

class Vector2d
{
public:
  Vector2d(double x,double y)
  {
    Set(x,y);
  };

  double GetX(void) const { return mX; };

  double GetY(void) const { return mY; };

  void  Set(double x,double y)
  {
    mX = x;
    mY = y;
  };
private:
  double mX;
  double mY;
};

// Typedef an STL vector of vertices which are used to represent
// a polygon/contour and a series of triangles.
//typedef std::vector< Vector2d > MyVectorOfVertices;
//typedef std::vector< Vector3d > MyVectorOfVertices;
typedef std::vector< TVec5d > MyVectorOfVertices; // TVec5d (for 3D + UV) // MT


class Triangulate
{
public:

  // triangulate a contour/polygon, places results in STL vector
  // as series of triangles.
  static bool Process(const MyVectorOfVertices &contour,
                      MyVectorOfVertices &result);

  // compute area of a contour/polygon
  static double Area(const MyVectorOfVertices &contour);

  // decide if point Px/Py is inside triangle defined by
  // (Ax,Ay) (Bx,By) (Cx,Cy)
  static bool InsideTriangle(double Ax, double Ay,
                      double Bx, double By,
                      double Cx, double Cy,
                      double Px, double Py);


private:
  static bool Snip(const MyVectorOfVertices &contour,int u,int v,int w,int n,int *V);

};


#endif