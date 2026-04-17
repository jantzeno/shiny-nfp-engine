#pragma once

#include <memory>

#include <spdlog/spdlog.h>

namespace shiny::log {

void InitializeLogger();
void ShutdownLogger();
std::shared_ptr<spdlog::logger> GetLogger();

} // namespace shiny::log

#define SHINY_TRACE(...) (::shiny::log::GetLogger()->trace(__VA_ARGS__))
#define SHINY_DEBUG(...) (::shiny::log::GetLogger()->debug(__VA_ARGS__))
#define SHINY_INFO(...) (::shiny::log::GetLogger()->info(__VA_ARGS__))
#define SHINY_WARN(...) (::shiny::log::GetLogger()->warn(__VA_ARGS__))
#define SHINY_ERROR(...) (::shiny::log::GetLogger()->error(__VA_ARGS__))
