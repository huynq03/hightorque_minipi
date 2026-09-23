#include "rl/PolicyPackage.h"

#include <algorithm>
#include <filesystem>

namespace mini_pi
{
namespace rl
{
namespace fs = std::filesystem;

namespace
{
std::string join(const fs::path& a, const char* b, const char* c)
{
    return (a / b / c).string();
}
} // namespace

bool isPolicyPackage(const std::string& dir, std::string* why)
{
    std::error_code ec;
    if (!fs::is_directory(dir, ec))
    {
        if (why) *why = dir + " is not a directory";
        return false;
    }
    const fs::path p(dir);
    const std::string yaml  = join(p, kParamsSubdir, kDeployYamlName);
    const std::string model = join(p, kExportedSubdir, kModelName);

    // Both are required. Accepting on `exported/` alone -- as Unitree does --
    // means a package missing its deploy.yaml resolves successfully and then
    // throws from YAML::LoadFile deep inside construction.
    if (!fs::is_regular_file(yaml, ec))
    {
        if (why) *why = "missing " + yaml;
        return false;
    }
    if (!fs::is_regular_file(model, ec))
    {
        if (why) *why = "missing " + model;
        return false;
    }
    return true;
}

namespace
{
PolicyPackage accept(const fs::path& dir)
{
    PolicyPackage pkg;
    pkg.dir         = dir.string();
    pkg.deploy_yaml = join(dir, kParamsSubdir, kDeployYamlName);
    pkg.model       = join(dir, kExportedSubdir, kModelName);
    pkg.version     = dir.filename().string();
    pkg.valid       = true;
    return pkg;
}

PolicyPackage reject(std::string why)
{
    PolicyPackage pkg;
    pkg.error = std::move(why);
    return pkg;
}
} // namespace

PolicyPackage resolvePolicyPackage(const std::string& policy_dir,
                                   const std::string& base_dir,
                                   const std::string& version)
{
    if (policy_dir.empty()) return reject("policy_dir is empty");

    fs::path root(policy_dir);
    if (root.is_relative() && !base_dir.empty()) root = fs::path(base_dir) / root;

    std::error_code ec;
    if (!fs::is_directory(root, ec))
        return reject("policy_dir '" + root.string() + "' does not exist");

    // --- explicit version -------------------------------------------------
    if (!version.empty())
    {
        const fs::path cand = root / version;
        std::string why;
        if (!isPolicyPackage(cand.string(), &why))
            return reject("policy version '" + version + "' is not a valid package: " + why);
        return accept(cand);
    }

    // --- policy_dir is itself a package -----------------------------------
    if (isPolicyPackage(root.string())) return accept(root);

    // --- otherwise: newest valid version ----------------------------------
    std::vector<fs::path> candidates;
    for (const auto& entry : fs::directory_iterator(root, ec))
    {
        if (entry.is_directory(ec)) candidates.push_back(entry.path());
    }
    if (candidates.empty())
        return reject("policy_dir '" + root.string() +
                      "' is not a policy package and contains no version directories "
                      "(expected " + kParamsSubdir + "/" + kDeployYamlName + " and " +
                      kExportedSubdir + "/" + kModelName + ")");

    // Lexicographic, same rule as Unitree's parser_policy_dir. Scanning
    // backwards makes the last name win, so v0 < v1 < v2. NOTE: this is a
    // STRING sort, so v10 sorts before v9 -- zero-pad beyond v9.
    std::sort(candidates.begin(), candidates.end());

    std::string rejected;
    for (auto it = candidates.rbegin(); it != candidates.rend(); ++it)
    {
        std::string why;
        if (isPolicyPackage(it->string(), &why)) return accept(*it);
        rejected += (rejected.empty() ? "" : "; ") + why;
    }
    return reject("no valid policy package under '" + root.string() + "' (" + rejected + ")");
}

} // namespace rl
} // namespace mini_pi
