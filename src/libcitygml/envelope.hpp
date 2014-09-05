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
#ifndef __CITYGML_ENVELOPE_HPP__
#define __CITYGML_ENVELOPE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "vecs.hpp"
#include <ostream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class Envelope
{
    friend class CityGMLHandler;
public:
    Envelope();
    Envelope(const TVec3d& lowerBound, const TVec3d& upperBound);

    const TVec3d& getLowerBound() const;
    const TVec3d& getUpperBound() const;

    void merge(const Envelope& e);

    void merge(const TVec3d& p);

protected:
    TVec3d _lowerBound;
    TVec3d _upperBound;
};
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<( std::ostream&, const citygml::Envelope& );
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_ENVELOPE_HPP__
