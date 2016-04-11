#include "tag.hpp"

namespace documentADE
{
  std::string getText()
  {
      return _text;
  }
  int getCount()
  {
      return _count;
  }

  void setText(std::string text)
  {
      this._text=text;
  }
  void setCount(int count)
  {
      this._count=count;
  }
}
