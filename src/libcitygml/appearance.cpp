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
#include "appearance.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
  ////////////////////////////////////////////////////////////////////////////////
  Appearance::Appearance( const std::string& id, const std::string& typeString )
    : Object( id ), _typeString( typeString ), _isFront(true)
  {}
  ////////////////////////////////////////////////////////////////////////////////
  Appearance::~Appearance( void )
  {}
  ////////////////////////////////////////////////////////////////////////////////
  std::string Appearance::getType( void ) const
  {
    return _typeString;
  }
  ////////////////////////////////////////////////////////////////////////////////
  bool Appearance::getIsFront( void ) const
  {
    return _isFront;
  }
  ////////////////////////////////////////////////////////////////////////////////
  std::string Appearance::toString( void ) const
  {
    return _typeString + " " + _id;
  }
  ////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
