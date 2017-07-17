// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "tag.hpp"

namespace documentADE
{
  Tag::Tag( const std::string& id ) : Object( id )
  {
  }
  std::string Tag::getText()
  {
      return _text;
  }
  int Tag::getCount()
  {
      return _count;
  }

  void Tag::setText(std::string text)
  {
      _text=text;
  }
  void Tag::setCount(int count)
  {
      _count=count;
  }
}
