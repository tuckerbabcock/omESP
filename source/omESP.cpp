#include <string>

#include "omESP/omESP.hpp"

exported_class::exported_class()
    : m_name("omESP")
{
}

auto exported_class::name() const -> const char*
{
  return m_name.c_str();
}
