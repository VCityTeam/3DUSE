#ifndef TAG_HPP
#define TAG_HPP

#include "cityobject.hpp"

namespace documentADE
{
  class Tag: public citygml::Object
  {
  public:
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

