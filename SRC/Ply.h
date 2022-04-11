/*

 Header for PLY polygon files.

  - Greg Turk, March 1994

   A PLY file contains a single polygonal _object_.

	An object is composed of lists of _elements_.  Typical elements are
	vertices, faces, edges and materials.

	 Each type of element for a given object has one or more _properties_
	 associated with the element type.  For instance, a vertex element may
	 have as properties three floating-point values x,y,z and three unsigned
	 chars for red, green and blue.

	  ---------------------------------------------------------------

	   Copyright (c) 1994 The Board of Trustees of The Leland Stanford
	   Junior University.  All rights reserved.

		Permission to use, copy, modify and distribute this software and its
		documentation for any purpose is hereby granted without fee, provided
		that the above copyright notice and this permission notice appear in
		all copies of this software and that you do not sell the software.

		 THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,
		 EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY
		 WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.

*/

#ifndef PLY_INCLUDED
#define PLY_INCLUDED
#include "Geometry.h"
#include <vector>

class PlyVertex
{
public:
	const static int Components=3;
	Point3D<float> point;

	operator Point3D<float>& ()					{return point;}
	operator const Point3D<float>& () const		{return point;}
	PlyVertex(void)								{point[0]=point[1]=point[2]=0;}
	PlyVertex(const Point3D<float>& p)			{point=p;}
};

#endif // PLY_INCLUDED
