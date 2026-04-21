#pragma once

#include "cache/nfp_cache.hpp"

namespace shiny::nesting::pack {

struct PackerWorkspace {
  cache::NfpCache nfp_cache{cache::default_nfp_cache_config()};
};

} // namespace shiny::nesting::pack
