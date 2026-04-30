set_project("shiny-nesting-engine")
set_version("0.1.0")

set_languages("cxx20")
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
    add_headerfiles(
        "src/algorithm_kind.hpp",
        "src/api/(**.hpp)",
        "src/observer.hpp",
        "src/request.hpp",
        "src/result.hpp",
        "src/solve.hpp",
        "src/cache/(**.hpp)",
        "src/geometry/(**.hpp)",
        "src/io/(**.hpp)",
        "src/logging/(**.hpp)",
        "src/nfp/(**.hpp)",
        "src/packing/(**.hpp)",
        "src/placement/(**.hpp)",
        "src/predicates/(**.hpp)",
        "src/runtime/(**.hpp)",
        "src/search/(**.hpp)",
        "src/validation/(**.hpp)",
        "src/util/(**.hpp)")
    add_files(
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
        "src/packing/bounding_box/*.cpp",
        "src/packing/irregular/*.cpp",
        "src/packing/irregular/sequential/*.cpp",
        "src/placement/config.cpp",
        "src/geometry/operations/*.cpp",
        "src/predicates/*.cpp",
        "src/runtime/*.cpp",
        "src/validation/*.cpp")
    add_files("src/search/*.cpp")
    add_files("src/search/detail/*.cpp")
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
    add_files("tests/unit/api_surface.cpp")
    add_files("tests/unit/aabb_prerejection.cpp")
    add_files("tests/unit/boost_geometry_smoke.cpp")
    add_files("tests/unit/bounding_box_packer.cpp")
    add_files("tests/unit/cache.cpp")
    add_files("tests/unit/candidate_generation.cpp")
    add_files("tests/unit/constructive_irregular.cpp")
    add_files("tests/unit/convex_nfp_collinear.cpp")
    add_files("tests/unit/cutting.cpp")
    add_files("tests/unit/decomposition_internal.cpp")
    add_files("tests/unit/execution_runtime.cpp")
    add_files("tests/unit/geometry_enhancements.cpp")
    add_files("tests/unit/geometry_foundation.cpp")
    add_files("tests/unit/geometry_normalize.cpp")
    add_files("tests/unit/import_preprocess.cpp")
    add_files("tests/unit/io_json.cpp")
    add_files("tests/unit/io_svg_polygonize.cpp")
    add_files("tests/unit/metaheuristic_search.cpp")
    add_files("tests/unit/nfp_geometry.cpp")
    add_files("tests/unit/or_dataset_json.cpp")
    add_files("tests/unit/overlap_infrastructure.cpp")
    add_files("tests/unit/current_surface_contracts.cpp")
    add_files("tests/unit/polygon_ops.cpp")
    add_files("tests/unit/predicates.cpp")
    add_files("tests/unit/production_search.cpp")
    add_files("tests/unit/readme_examples.cpp")
    add_files("tests/unit/request_normalization.cpp")
    add_files("tests/unit/separator.cpp")
    add_files("tests/unit/sequential_backtrack.cpp")
    add_files("tests/unit/strategy_registry.cpp")
    add_files("tests/unit/strip_optimizer.cpp")
    add_files("tests/unit/util.cpp")
    add_files("tests/readiness/readiness_matrix.cpp")
    add_files("tests/topology/polygon_union.cpp")
    add_files("tests/support/mtg_fixture.cpp")
    add_files("tests/integration/mtg_bounding_box_engine_surface.cpp")
    add_files("tests/integration/metaheuristic_search.cpp")
    add_files("tests/integration/representative_layouts.cpp")
    add_files("tests/integration/mtg_bb_heuristics.cpp")
    add_files("tests/integration/mtg_bin_assignment.cpp")
    add_files("tests/integration/mtg_engine_bug_repros.cpp")
    add_files("tests/integration/mtg_exclusion_zones.cpp")
    add_files("tests/integration/mtg_limits.cpp")
    add_files("tests/integration/mtg_margins.cpp")
    add_files("tests/integration/mtg_seeds.cpp")
    add_files("tests/integration/mtg_sliders.cpp")
    add_files("tests/integration/mtg_start_corners.cpp")
    add_files("tests/integration/mtg_test_nesting_matrix.cpp")
    add_files("tests/integration/public_surface_manifest.cpp")
    add_files("tests/integration/sequential_backtrack.cpp")
    add_sysincludedirs(nanosvg_dir)
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()
    add_tests("default")