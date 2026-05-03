set_project("shiny-nesting-engine")
set_version("0.1.0")

set_languages("cxx23")
set_warnings("allextra", "error")

local workspace_root = path.absolute(path.join(os.projectdir(), ".."))
local workspace_compile_commands_script = path.join(workspace_root, "merge_compile_commands.py")
local workspace_compile_commands_output = path.join(workspace_root, "compile_commands.json")
local workspace_compile_commands_merged = false

rule("workspace.merge_compile_commands")
    after_build(function(target)
        if target:kind() == "phony"
            or workspace_compile_commands_merged
            or not os.isfile(workspace_compile_commands_script) then
            return
        end

        workspace_compile_commands_merged = true
        os.execv("python3", {
            workspace_compile_commands_script,
            "--workspace-root", workspace_root,
            "--output", workspace_compile_commands_output
        })
    end)
rule_end()

add_rules("mode.debug", "mode.release")
add_rules("plugin.compile_commands.autoupdate", {outputdir = ".vscode", lsp = "clangd"})
add_rules("workspace.merge_compile_commands")

target("merge_workspace_compile_commands")
    set_kind("phony")
    set_default(false)
    on_build(function()
        if workspace_compile_commands_merged or not os.isfile(workspace_compile_commands_script) then
            return
        end

        workspace_compile_commands_merged = true
        os.execv("python3", {
            workspace_compile_commands_script,
            "--workspace-root", workspace_root,
            "--output", workspace_compile_commands_output
        })
    end)

option("sanitizer")
    set_default("none")
    set_showmenu(true)
    set_values("none", "address", "undefined", "address-undefined")
option_end()

local boost_root = "vendor/boost"
local earcut_include_dir = "vendor/earcut.hpp/include"
local spdlog_include_dir = "vendor/spdlog-1.17.0/include"
local nanosvg_dir = "vendor/nanosvg/src"
local fixture_root = path.translate(path.join(os.projectdir(), "tests", "fixtures"))

local function selected_sanitizer()
    local sanitizer = os.getenv("SHINY_NESTING_SANITIZER")
    if sanitizer ~= nil and sanitizer ~= "" then
        return sanitizer
    end

    sanitizer = get_config("sanitizer")
    if sanitizer == nil or sanitizer == "" then
        return "none"
    end
    return sanitizer
end

local function sanitizer_flag_value()
    local sanitizer = selected_sanitizer()
    if sanitizer == "address" then
        return "address"
    end
    if sanitizer == "undefined" then
        return "undefined"
    end
    if sanitizer == "address-undefined" then
        return "address,undefined"
    end
    return nil
end

local function add_sanitizer_compile_flags()
    local sanitizer = sanitizer_flag_value()
    if sanitizer ~= nil then
        add_cxflags("-fno-omit-frame-pointer", {force = true})
        add_cxflags("-fsanitize=" .. sanitizer, {force = true})
    end
end

local function add_sanitizer_link_flags()
    local sanitizer = sanitizer_flag_value()
    if sanitizer ~= nil then
        add_ldflags("-fsanitize=" .. sanitizer, {force = true})
    end
end

local function add_thread_support_flags()
    add_cxflags("-pthread", {force = true})
    add_ldflags("-pthread", {force = true})
end

local function add_vendor_warning_suppression_flags()
    if is_plat("linux") then
        add_cxflags("-Wno-error=maybe-uninitialized", {force = true})
    end
end

package("vendored_boost")
    set_kind("library", {headeronly = true})
    on_fetch(function ()
        return {
            sysincludedirs = {boost_root}
        }
    end)
package_end()

add_requires("catch2")

target("vendor_spdlog")
    set_kind("headeronly")
    set_default(false)
    add_includedirs(spdlog_include_dir, {public = true})

target("shiny_logging")
    set_kind("static")
    set_default(false)
    set_warnings("all")
    add_includedirs("src", {public = true})
    add_files("src/logging/*.cpp")
    add_deps("vendor_spdlog")

target("shiny_nesting_engine")
    set_kind("static")
    add_packages("vendored_boost", {public = true})
    add_includedirs("src", {public = true})
    add_sysincludedirs(boost_root, {public = true})
    add_sysincludedirs(earcut_include_dir)
    -- Stable public entry points plus transitive headers still required by the
    -- current request/result surface. Milestone 7 tracks shrinking this list as
    -- backend-only packing types are pushed back behind public DTO/request
    -- boundaries.
    --
    -- Internal headers under src/internal/ are NOT exported:
    --   - src/internal/legacy_strategy.hpp  (strategy-era enums and configs)
    --   - src/internal/legacy_solve.hpp     (NestingRequest solve overload)
    --   - src/internal/request_normalization.hpp (normalized request details)
    -- These are transitively included by request.hpp / solve.cpp but are not
    -- part of the supported downstream API surface.
    add_headerfiles(
        "src/algorithm_kind.hpp",
        "src/api/dto.hpp",
        "src/api/profiles.hpp",
        "src/api/request_builder.hpp",
        "src/api/solve_control.hpp",
        "src/observer.hpp",
        "src/request.hpp",
        "src/result.hpp",
        "src/solve.hpp",
        "src/geometry/concepts.hpp",
        "src/geometry/polygon.hpp",
        "src/geometry/types.hpp",
        "src/geometry/vector_ops.hpp",
        "src/geometry/operations/*.hpp",
        "src/geometry/queries/*.hpp",
        "src/geometry/transforms/*.hpp",
        "src/placement/(**.hpp)",
        "src/runtime/cancellation.hpp",
        "src/runtime/progress.hpp",
        "src/util/status.hpp")
    add_files(
        "src/api/*.cpp",
        "src/request.cpp",
        "src/solve.cpp",
        "src/cache/*.cpp",
        "src/geometry/*.cpp",
        "src/geometry/decomposition/*.cpp",
        "src/geometry/queries/*.cpp",
        "src/geometry/transforms/*.cpp",
        "src/io/*.cpp",
        "src/nfp/*.cpp",
        "src/packing/*.cpp",
        "src/packing/constructive/*.cpp",
        "src/packing/sparrow/*.cpp",
        "src/packing/sparrow/adapters/*.cpp",
        "src/packing/sparrow/eval/*.cpp",
        "src/packing/sparrow/optimize/*.cpp",
        "src/packing/sparrow/quantify/*.cpp",
        "src/packing/sparrow/runtime/*.cpp",
        "src/packing/sparrow/search/brkga_search.cpp",
        "src/packing/sparrow/search/disruption.cpp",
        "src/packing/sparrow/search/solution_pool.cpp",
        "src/packing/sparrow/search/strip_optimizer.cpp",
        "src/packing/sparrow/search/detail/*.cpp",
        "src/packing/sparrow/sample/*.cpp",
        "src/packing/bounding_box/*.cpp",
        "src/packing/irregular/*.cpp",
        "src/packing/irregular/sequential/*.cpp",
        "src/placement/config.cpp",
        "src/geometry/operations/*.cpp",
        "src/predicates/*.cpp",
        "src/runtime/*.cpp",
        "src/validation/*.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()

target("shiny_nesting_engine_tests")
    set_kind("binary")
    before_build(function()
        os.execv("python3", {"scripts/check-test-manifest.py"})
    end)
    add_packages("catch2", "vendored_boost")
    add_deps("shiny_nesting_engine")
    add_defines('SHINY_NESTING_ENGINE_TEST_FIXTURE_ROOT="' .. fixture_root .. '"')
    add_includedirs("tests")
    add_sysincludedirs(boost_root)
    add_sysincludedirs(earcut_include_dir)
    add_files("tests/unit/api/api_surface.cpp")
    add_files("tests/unit/api/current_surface_contracts.cpp")
    add_files("tests/unit/api/profiles.cpp")
    add_files("tests/unit/api/readme_examples.cpp")
    add_files("tests/unit/geometry/aabb_prerejection.cpp")
    add_files("tests/unit/geometry/boost_geometry_smoke.cpp")
    add_files("tests/unit/packing/bounding_box_packer.cpp")
    add_files("tests/unit/packing/cache.cpp")
    add_files("tests/unit/packing/candidate_generation.cpp")
    add_files("tests/unit/constructive/constructive_irregular.cpp")
    add_files("tests/unit/geometry/convex_nfp_collinear.cpp")
    add_files("tests/unit/packing/cutting.cpp")
    add_files("tests/unit/geometry/decomposition_internal.cpp")
    add_files("tests/unit/runtime/execution_runtime.cpp")
    add_files("tests/unit/runtime/layout_validation.cpp")
    add_files("tests/unit/runtime/legacy_strategy.cpp")
    add_files("tests/unit/geometry/geometry_enhancements.cpp")
    add_files("tests/unit/geometry/geometry_foundation.cpp")
    add_files("tests/unit/geometry/geometry_normalize.cpp")
    add_files("tests/unit/io/import_preprocess.cpp")
    add_files("tests/unit/io/io_json.cpp")
    add_files("tests/unit/io/io_svg_polygonize.cpp")
    add_files("tests/unit/sparrow/metaheuristic_search.cpp")
    add_files("tests/unit/api/nesting_api_smoke.cpp")
    add_files("tests/unit/constructive/fill_first_ordering_contracts.cpp")
    add_files("tests/unit/geometry/nfp_geometry.cpp")
    add_files("tests/unit/io/or_dataset_json.cpp")
    add_files("tests/unit/packing/overlap_infrastructure.cpp")
    add_files("tests/unit/packing/packing_scoring.cpp")
    add_files("tests/unit/packing/blocked_regions.cpp")
    add_files("tests/unit/geometry/polygon_ops.cpp")
    add_files("tests/unit/geometry/predicates.cpp")
    add_files("tests/unit/sparrow/production_search.cpp")
    add_files("tests/unit/api/request_normalization.cpp")
    add_files("tests/unit/packing/separator.cpp")
    add_files("tests/unit/sparrow/sparrow_constructive_seed.cpp")
    add_files("tests/unit/sparrow/sparrow_quantify.cpp")
    add_files("tests/unit/sparrow/sparrow_sampling.cpp")
    add_files("tests/unit/sparrow/sparrow_separator.cpp")
    add_files("tests/unit/sparrow/sparrow_tracker.cpp")
    add_files("tests/unit/sparrow/strip_optimizer.cpp")
    add_files("tests/unit/runtime/util.cpp")
    add_files("tests/readiness/readiness_matrix.cpp")
    add_files("tests/readiness/sparrow_readiness_matrix.cpp")
    add_files("tests/topology/polygon_union.cpp")
    add_files("tests/fixtures/export_surface/mtg_fixture.cpp")
    add_files("tests/integration/export_surface/mtg_bounding_box_engine_surface.cpp")
    add_files("tests/integration/sparrow/metaheuristic_search.cpp")
    add_files("tests/integration/sparrow/representative_layouts.cpp")
    add_files("tests/integration/export_surface/mtg_bb_heuristics.cpp")
    add_files("tests/integration/export_surface/mtg_bin_assignment.cpp")
    add_files("tests/integration/export_surface/mtg_engine_bug_repros.cpp")
    add_files("tests/integration/export_surface/mtg_exclusion_zones.cpp")
    add_files("tests/integration/export_surface/mtg_limits.cpp")
    add_files("tests/integration/export_surface/mtg_margins.cpp")
    add_files("tests/integration/export_surface/mtg_seeds.cpp")
    add_files("tests/integration/export_surface/mtg_sliders.cpp")
    add_files("tests/integration/export_surface/mtg_sliders_reject.cpp")
    add_files("tests/integration/export_surface/mtg_start_corners.cpp")
    add_files("tests/integration/export_surface/mtg_test_nesting_matrix.cpp")
    add_files("tests/integration/constructive/fill_first_engine_integration.cpp")
    add_files("tests/integration/export_surface/public_surface_manifest.cpp")
    add_files("tests/integration/sparrow/multi_bin.cpp")
    add_files("tests/integration/sparrow/knapsack.cpp")
    add_files("tests/integration/sparrow/sparrow_assignment_overflow.cpp")
    add_files("tests/integration/profiles/balanced.cpp")
    add_files("tests/integration/profiles/maximum_search.cpp")
    add_sysincludedirs(nanosvg_dir)
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()
    add_tests("default")