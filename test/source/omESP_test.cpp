#include <string>

#include "omESP/omESP.hpp"

auto main() -> int
{
  exported_class e;

  return std::string("omESP") == e.name() ? 0 : 1;
}
