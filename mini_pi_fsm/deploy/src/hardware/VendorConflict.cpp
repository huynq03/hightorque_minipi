#include "hardware/VendorConflict.h"

#include <dirent.h>
#include <unistd.h>

#include <climits>
#include <cstdlib>
#include <fstream>
#include <iterator>

namespace mini_pi
{
namespace
{
std::vector<std::string> readCmdline(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    std::string raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::vector<std::string> args;
    std::string cur;
    for (char ch : raw)
    {
        if (ch == '\0') { args.push_back(cur); cur.clear(); }
        else cur.push_back(ch);
    }
    if (!cur.empty()) args.push_back(cur);
    return args;
}

std::string basename_of(const std::string& p)
{
    const auto s = p.find_last_of('/');
    return s == std::string::npos ? p : p.substr(s + 1);
}

bool isPid(const char* name)
{
    if (!*name) return false;
    for (const char* c = name; *c; ++c)
        if (*c < '0' || *c > '9') return false;
    return true;
}
} // namespace

std::vector<VendorConflict> findVendorConflicts(const std::string& proc_root, int self_pid)
{
    static const char* kExecutables[] = {"sim2real_master_node", "lr_control_node",
                                         "rl_pd_controller", "hightorque_hardware_sdk_node"};
    static const char* kLaunchWords[] = {"sim2real_master", "joy_control_pi", "sim2real.launch"};

    std::vector<VendorConflict> out;
    DIR* d = opendir(proc_root.c_str());
    if (!d) return out;

    while (dirent* e = readdir(d))
    {
        if (!isPid(e->d_name)) continue;
        const int pid = std::atoi(e->d_name);
        if (pid == self_pid) continue;
        const std::string dir = proc_root + "/" + e->d_name;

        const auto args = readCmdline(dir + "/cmdline");
        if (!args.empty())
        {
            const std::string exe = basename_of(args[0]);
            bool hit = false;
            for (const char* x : kExecutables)
                if (exe == x) { out.push_back({pid, "vendor controller '" + exe + "' is running"}); hit = true; break; }
            if (!hit && args[0].find("livelybot_bringup/") != std::string::npos)
            {
                out.push_back({pid, "livelybot_bringup tool '" + exe + "' is running"});
                hit = true;
            }
            if (!hit)
            {
                // roslaunch is a python script: argv[0] may be python, argv[1] roslaunch.
                bool is_roslaunch = false;
                for (std::size_t i = 0; i < args.size() && i < 3; ++i)
                    if (basename_of(args[i]) == "roslaunch") is_roslaunch = true;
                if (is_roslaunch)
                {
                    for (const auto& a : args)
                    {
                        for (const char* w : kLaunchWords)
                        {
                            if (a.find(w) != std::string::npos)
                            {
                                out.push_back({pid, "vendor roslaunch ('" + a + "') is running"});
                                hit = true;
                                break;
                            }
                        }
                        if (hit) break;
                    }
                }
            }
            if (hit) continue;
        }

        // Open motor-serial file descriptors.
        const std::string fddir = dir + "/fd";
        DIR* fdd = opendir(fddir.c_str());
        if (!fdd) continue;
        while (dirent* fe = readdir(fdd))
        {
            if (!isPid(fe->d_name)) continue;
            char buf[PATH_MAX];
            const ssize_t n = readlink((fddir + "/" + fe->d_name).c_str(), buf, sizeof(buf) - 1);
            if (n <= 0) continue;
            buf[n] = '\0';
            const std::string target(buf);
            if (target.rfind("/dev/ttyACM", 0) == 0)
            {
                const std::string exe = args.empty() ? std::string("?") : basename_of(args[0]);
                out.push_back({pid, "'" + exe + "' holds motor serial " + target + " open"});
                break;
            }
        }
        closedir(fdd);
    }
    closedir(d);
    return out;
}

} // namespace mini_pi
