#include "tag.hpp"

namespace documentADE
{
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
