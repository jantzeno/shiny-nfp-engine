#include "cache/stores.hpp"

#include "nfp/cache_keys.hpp"
#include "nfp/nonconvex_nfp.hpp"
#include "nfp/types.hpp"

template class shiny::nfp::cache::CacheStore<shiny::nfp::cache::PairRotationKey,
                                             shiny::nfp::NfpResult>;
template class shiny::nfp::cache::CacheStore<
    shiny::nfp::cache::NonconvexNfpCacheKey, shiny::nfp::NonconvexNfpResult>;