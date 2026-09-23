// rl::PolicyPackage -- locating and validating one policy package on disk.
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/param.h
// `parser_policy_dir()` (lines 86-115), which this reimplements with three
// deliberate additions, all of them required by the task:
//
//   1. an explicit version may be named, not only "the newest";
//   2. a directory is only ACCEPTED if it contains BOTH params/deploy.yaml and
//      exported/policy.onnx -- Unitree accepts on the presence of `exported/`
//      alone, which turns a half-copied package into a runtime crash;
//   3. every rejection carries a reason string, so an invalid package fails
//      startup with an explanation instead of silently resolving to the parent.
//
// A package is:
//     <dir>/params/deploy.yaml
//     <dir>/exported/policy.onnx
//
// A package DIRECTORY (the thing `policy_dir` normally points at) is a
// directory of such packages, conventionally named v0, v1, v2 ...
#pragma once

#include <string>
#include <vector>

namespace mini_pi
{
namespace rl
{

struct PolicyPackage
{
    std::string dir;          ///< the resolved package directory
    std::string deploy_yaml;  ///< <dir>/params/deploy.yaml
    std::string model;        ///< <dir>/exported/policy.onnx
    std::string version;      ///< the directory's own name, e.g. "v0"

    bool valid = false;
    /// Why resolution failed. Empty when valid.
    std::string error;
};

/// Filenames are fixed by the Unitree convention and are not configurable.
constexpr const char* kParamsSubdir   = "params";
constexpr const char* kDeployYamlName = "deploy.yaml";
constexpr const char* kExportedSubdir = "exported";
constexpr const char* kModelName      = "policy.onnx";

/// Resolves `policy_dir` to a single package.
///
///   - `policy_dir` relative      -> resolved against `base_dir`.
///   - `policy_dir` IS a package  -> used directly.
///   - otherwise                  -> its subdirectories are sorted
///                                   lexicographically and scanned from the
///                                   last backwards; the first VALID package
///                                   wins. Invalid candidates are recorded in
///                                   `error` so a typo is visible.
///   - `version` non-empty        -> only `<policy_dir>/<version>` is
///                                   considered, and it must be valid.
///
/// Never throws. Check `.valid`; `.error` says why not.
PolicyPackage resolvePolicyPackage(const std::string& policy_dir,
                                   const std::string& base_dir,
                                   const std::string& version = std::string());

/// True when `dir` has both required files. `why` receives the first missing
/// one when it returns false.
bool isPolicyPackage(const std::string& dir, std::string* why = nullptr);

} // namespace rl
} // namespace mini_pi
