// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef TAG_HPP
#define TAG_HPP

#include "cityobject.hpp"

namespace documentADE
{
  class Tag: public citygml::Object
  {
  public:
      Tag(const std::string& id);
      std::string getText();
      int getCount();

      void setText(std::string text);
      void setCount(int count);

  private:
      int _count;
      std::string _text;
  };
}

#endif // TAG_HPP

