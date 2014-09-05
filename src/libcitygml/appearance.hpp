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
#ifndef __CITYGML_APPEARANCE_HPP__
#define __CITYGML_APPEARANCE_HPP__
////////////////////////////////////////////////////////////////////////////////
#include "object.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
class Appearance : public Object
{
    friend class CityGMLHandler;
public:
    Appearance( const std::string& id, const std::string& typeString );

    virtual ~Appearance( void );

    std::string getType( void ) const;
    bool getIsFront( void ) const;

    virtual std::string toString( void ) const;

protected:
    std::string _typeString;
    bool _isFront;
};
////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
#endif // __CITYGML_APPEARANCE_HPP__
