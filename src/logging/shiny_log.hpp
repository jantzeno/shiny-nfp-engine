#pragma once

#include <memory>

#include <spdlog/spdlog.h>

namespace shiny::log {

void InitializeLogger();
void ShutdownLogger();
std::shared_ptr<spdlog::logger> GetLogger();

} // namespace shiny::log