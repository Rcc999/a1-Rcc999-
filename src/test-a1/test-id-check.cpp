#include <fstream>
#include <filesystem>
#include <regex>
#include <sstream>
#include <cstdio>
#include <vector>

#include "TestResult.h"

using namespace tests;

namespace {

std::string read_file_to_string(const std::string& path) {
    namespace fs = std::filesystem;
    std::vector<fs::path> candidates;
    candidates.emplace_back(path);

    fs::path cur = fs::current_path();
    for (int i = 0; i < 8; ++i) {
        candidates.push_back(cur / path);
        if (!cur.has_parent_path()) break;
        cur = cur.parent_path();
    }

    for (const auto& p : candidates) {
        std::ifstream file(p);
        if (!file.is_open()) continue;
        std::ostringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }
    return "";
}

std::string extract_json_string_value(const std::string& json,
                                      const std::string& key) {
    auto regex_escape = [](const std::string& s) {
        static const std::regex metacharacters(R"([-[\]{}()*+?.,\^$|#\s])");
        return std::regex_replace(s, metacharacters, R"(\$&)");
    };

    const std::regex pattern("\"" + regex_escape(key) + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch match;
    if (std::regex_search(json, match, pattern) && match.size() >= 2) {
        return match[1].str();
    }
    return "";
}

std::string shell_escape_single_quotes(const std::string& input) {
    std::string escaped = "'";
    for (char c : input) {
        if (c == '\'') escaped += "'\"'\"'";
        else escaped += c;
    }
    escaped += "'";
    return escaped;
}

bool is_url_reachable(const std::string& url) {
#ifdef WIN32
    const std::string discard_target = "NUL";
#else
    const std::string discard_target = "/dev/null";
#endif
    const std::string cmd =
        "curl -I -L -sS --max-time 10 -o " + discard_target + " -w \"%{http_code}\" " +
        shell_escape_single_quotes(url) + " 2>/dev/null";

#ifdef WIN32
    FILE* pipe = _popen(cmd.c_str(), "r");
#else
    FILE* pipe = popen(cmd.c_str(), "r");
#endif
    if (!pipe) return false;

    char buffer[32] = {0};
    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) output += buffer;

#ifdef WIN32
    _pclose(pipe);
#else
    pclose(pipe);
#endif

    if (output.size() < 3) return false;
    const char c = output[0];
    return c == '2' || c == '3';
}

bool starts_with_http_scheme(const std::string& url) {
    return url.rfind("https://", 0) == 0 || url.rfind("http://", 0) == 0;
}

TestResult expect_true(bool condition, const std::string& message) {
    if (condition) return {};
    return {false, message + "\n"};
}

}  // namespace

TestResult test_id_check_my_info_json() {
    const std::string json = read_file_to_string("my-info.json");
    if (json.empty()) return {false, "Could not read my-info.json\n"};

    const std::string full_name = extract_json_string_value(json, "full name");
    const std::string student_id = extract_json_string_value(json, "student id");
    const std::string video_ex4 =
        extract_json_string_value(json, "Demo video URL for ex.4");
    const std::string video_ex5_1 =
        extract_json_string_value(json, "Demo video URL for ex.5.1 (MiniPi)");
    const std::string video_ex5_2 =
        extract_json_string_value(json, "Demo video URL for ex.5.2 (HexPod)");

    TestResult res;
    res += expect_true(!full_name.empty(), "Missing or invalid \"full name\" in my-info.json");
    res += expect_true(full_name != "John Doe",
                       "\"full name\" is still default (John Doe)");
    res += expect_true(!student_id.empty(), "Missing or invalid \"student id\" in my-info.json");
    res += expect_true(student_id != "12-345-678",
                       "\"student id\" is still default (12-345-678)");

    res += expect_true(!video_ex4.empty(),
                       "Missing or invalid \"Demo video URL for ex.4\" in my-info.json");
    res += expect_true(!video_ex5_1.empty(),
                       "Missing or invalid \"Demo video URL for ex.5.1 (MiniPi)\" in my-info.json");
    res += expect_true(!video_ex5_2.empty(),
                       "Missing or invalid \"Demo video URL for ex.5.2 (HexPod)\" in my-info.json");

    res += expect_true(video_ex4 != "www.youtube.com",
                       "\"Demo video URL for ex.4\" is still default (www.youtube.com)");
    res += expect_true(video_ex5_1 != "www.youtube.com",
                       "\"Demo video URL for ex.5.1 (MiniPi)\" is still default (www.youtube.com)");
    res += expect_true(video_ex5_2 != "www.youtube.com",
                       "\"Demo video URL for ex.5.2 (HexPod)\" is still default (www.youtube.com)");

    if (!video_ex4.empty()) {
        res += expect_true(starts_with_http_scheme(video_ex4),
                           "\"Demo video URL for ex.4\" must start with http:// or https://");
        if (starts_with_http_scheme(video_ex4)) {
            res += expect_true(is_url_reachable(video_ex4),
                               "\"Demo video URL for ex.4\" is not reachable on the internet");
        }
    }

    if (!video_ex5_1.empty()) {
        res += expect_true(starts_with_http_scheme(video_ex5_1),
                           "\"Demo video URL for ex.5.1 (MiniPi)\" must start with http:// or https://");
        if (starts_with_http_scheme(video_ex5_1)) {
            res += expect_true(is_url_reachable(video_ex5_1),
                               "\"Demo video URL for ex.5.1 (MiniPi)\" is not reachable on the internet");
        }
    }

    if (!video_ex5_2.empty()) {
        res += expect_true(starts_with_http_scheme(video_ex5_2),
                           "\"Demo video URL for ex.5.2 (HexPod)\" must start with http:// or https://");
        if (starts_with_http_scheme(video_ex5_2)) {
            res += expect_true(is_url_reachable(video_ex5_2),
                               "\"Demo video URL for ex.5.2 (HexPod)\" is not reachable on the internet");
        }
    }

    return res;
}

int main(int argc, char *argv[]) {
    TEST(test_id_check_my_info_json);
    return (allTestsOk ? 0 : 1);
}
