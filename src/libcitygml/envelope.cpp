/* -*-c++-*- libcitygml - Copyright (c) 2010 Joachim Pouderoux, BRGM
*
* This file is part of libcitygml library
* http://code.google.com/p/libcitygml
*
* libcitygml is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 2.1 of the License, or
* (at your option) any later version.
*
* libcitygml is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*/
////////////////////////////////////////////////////////////////////////////////
#include "envelope.hpp"
#include <limits>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
Envelope::Envelope()
: _lowerBound(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),std::numeric_limits<double>::max()),
  _upperBound(std::numeric_limits<double>::min(),std::numeric_limits<double>::min(),std::numeric_limits<double>::min())
{}
////////////////////////////////////////////////////////////////////////////////
Envelope::Envelope(const TVec3d& lowerBound, const TVec3d& upperBound)
    : _lowerBound(lowerBound), _upperBound(upperBound)
{
}
////////////////////////////////////////////////////////////////////////////////
const TVec3d& Envelope::getLowerBound() const
{
    return _lowerBound;
}
////////////////////////////////////////////////////////////////////////////////
const TVec3d& Envelope::getUpperBound() const
{
    return _upperBound;
}
////////////////////////////////////////////////////////////////////////////////
void Envelope::merge(const Envelope& e)
{
    merge(e._lowerBound);
    merge(e._upperBound);
}
////////////////////////////////////////////////////////////////////////////////
void Envelope::merge(const TVec3d& p)
{
    if(p.x < _lowerBound.x) _lowerBound.x = p.x;
    else if(p.x > _upperBound.x) _upperBound.x = p.x;

    if(p.y < _lowerBound.y) _lowerBound.y = p.y;
    else if(p.y > _upperBound.y) _upperBound.y = p.y;

    if(p.z < _lowerBound.z) _lowerBound.z = p.z;
    else if(p.z > _upperBound.z) _upperBound.z = p.z;
}
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream& os, const Envelope& e )
{
    return os << e.getLowerBound() << " " << e.getUpperBound();
}
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
