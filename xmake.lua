set_project("shiny-nfp-engine")
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
local cgal_root = "vendor/cgal"
local fixture_root = path.translate(path.join(os.projectdir(), "tests", "fixtures"))

local function collect_component_include_roots(root)
    local include_roots = {}

    if not os.isdir(root) then
        return include_roots
    end

    for _, dir in ipairs(os.dirs(path.join(root, "*/include"))) do
        table.insert(include_roots, dir)
    end

    table.sort(include_roots)
    table.insert(include_roots, root)
    return include_roots
end

local cgal_include_roots = collect_component_include_roots(cgal_root)

local function selected_sanitizer()
    local sanitizer = os.getenv("SHINY_NFP_SANITIZER")
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

package("vendored_cgal")
    set_kind("library", {headeronly = true})
    on_fetch(function ()
        return {
            sysincludedirs = cgal_include_roots
        }
    end)
package_end()

add_requires("catch2")

target("vendor_spdlog")
    set_kind("headeronly")
    set_default(false)
    add_includedirs("vendor/spdlog-1.17.0/include", {public = true})

target("shiny_logging")
    set_kind("static")
    set_default(false)
    set_warnings("all")
    add_includedirs("src", {public = true})
    add_files("src/logging/*.cpp")
    add_deps("vendor_spdlog")

target("shiny_nfp_engine")
    set_kind("static")
    add_packages("vendored_boost", "vendored_cgal")
    add_defines("CGAL_DO_NOT_USE_BOOST_MP=1", "CGAL_NO_GMP=1", "CGAL_NO_MPFR=1")
    add_includedirs("src", {public = true})
    add_sysincludedirs(boost_root)
    for _, dir in ipairs(cgal_include_roots) do
        add_sysincludedirs(dir)
    end
    add_headerfiles("src/(**.hpp)")
    add_files("src/**/*.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()

target("shiny_nfp_engine_tests")
    set_kind("binary")
    add_packages("catch2", "vendored_boost")
    add_deps("shiny_nfp_engine")
    add_defines('SHINY_NFP_ENGINE_TEST_FIXTURE_ROOT="' .. fixture_root .. '"')
    add_sysincludedirs(boost_root)
    add_includedirs("tests")
    add_includedirs("vendor/nanosvg/src")
    add_files("tests/**/*.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()
    add_tests("default")

target("shiny_nfp_engine_local_search_example")
    set_kind("binary")
    add_deps("shiny_nfp_engine")
    add_files("examples/local_search/main.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()

target("shiny_nfp_engine_genetic_search_example")
    set_kind("binary")
    add_deps("shiny_nfp_engine")
    add_files("examples/genetic_search/main.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()

target("shiny_nfp_engine_constructive_decoder_example")
    set_kind("binary")
    add_deps("shiny_nfp_engine")
    add_files("examples/constructive_decoder/main.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()

target("shiny_nfp_engine_masonry_builder_example")
    set_kind("binary")
    add_deps("shiny_nfp_engine")
    add_files("examples/masonry_builder/main.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()

target("shiny_nfp_engine_pairwise_nfp_example")
    set_kind("binary")
    add_deps("shiny_nfp_engine")
    add_files("examples/pairwise_nfp/main.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()

target("shiny_nfp_engine_cache_inspect")
    set_kind("binary")
    add_deps("shiny_nfp_engine")
    add_files("tools/cache_inspect/main.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()

target("shiny_nfp_engine_fixture_gen")
    set_kind("binary")
    add_deps("shiny_nfp_engine")
    add_files("tools/fixture_gen/main.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()

target("shiny_nfp_engine_profile_decode")
    set_kind("binary")
    add_deps("shiny_nfp_engine")
    add_files("tools/profile_decode/main.cpp")
    add_deps("shiny_logging")
    add_vendor_warning_suppression_flags()
    add_thread_support_flags()
    add_sanitizer_compile_flags()
    add_sanitizer_link_flags()
