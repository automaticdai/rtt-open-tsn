#pragma once

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace sim {

/**
 * Lightweight, zero-dependency RFC-8259 JSON value, parser, and serializer.
 */
class Json {
public:
    using Object = std::map<std::string, Json>;
    using Array = std::vector<Json>;
    using Value = std::variant<std::nullptr_t, bool, int64_t, double, std::string, Array, Object>;

    Json() : value_(nullptr) {}
    Json(std::nullptr_t) : value_(nullptr) {}
    Json(bool b) : value_(b) {}
    Json(int v) : value_(static_cast<int64_t>(v)) {}
    Json(int64_t v) : value_(v) {}
    Json(uint64_t v) : value_(static_cast<int64_t>(v)) {}
    Json(double v) : value_(v) {}
    Json(const char* s) : value_(std::string(s)) {}
    Json(std::string s) : value_(std::move(s)) {}
    Json(Array a) : value_(std::move(a)) {}
    Json(Object o) : value_(std::move(o)) {}

    bool is_null() const { return std::holds_alternative<std::nullptr_t>(value_); }
    bool is_bool() const { return std::holds_alternative<bool>(value_); }
    bool is_int() const { return std::holds_alternative<int64_t>(value_); }
    bool is_double() const { return std::holds_alternative<double>(value_); }
    bool is_number() const { return is_int() || is_double(); }
    bool is_string() const { return std::holds_alternative<std::string>(value_); }
    bool is_array() const { return std::holds_alternative<Array>(value_); }
    bool is_object() const { return std::holds_alternative<Object>(value_); }

    bool as_bool() const { return std::get<bool>(value_); }
    int64_t as_int() const {
        if (is_int()) return std::get<int64_t>(value_);
        if (is_double()) return static_cast<int64_t>(std::get<double>(value_));
        throw std::runtime_error("JSON value is not an integer");
    }
    double as_double() const {
        if (is_double()) return std::get<double>(value_);
        if (is_int()) return static_cast<double>(std::get<int64_t>(value_));
        throw std::runtime_error("JSON value is not a number");
    }
    const std::string& as_string() const { return std::get<std::string>(value_); }
    const Array& as_array() const { return std::get<Array>(value_); }
    Array& as_array() { return std::get<Array>(value_); }
    const Object& as_object() const { return std::get<Object>(value_); }
    Object& as_object() { return std::get<Object>(value_); }

    bool contains(const std::string& key) const {
        if (!is_object()) return false;
        const auto& obj = as_object();
        return obj.find(key) != obj.end();
    }

    const Json& at(const std::string& key) const {
        const auto& obj = as_object();
        auto it = obj.find(key);
        if (it == obj.end()) {
            throw std::runtime_error("JSON missing key: " + key);
        }
        return it->second;
    }

    template <typename T>
    T get_or(const std::string& key, T default_val) const {
        if (!contains(key)) return default_val;
        const auto& v = at(key);
        if constexpr (std::is_same_v<T, bool>) return v.as_bool();
        else if constexpr (std::is_integral_v<T>) return static_cast<T>(v.as_int());
        else if constexpr (std::is_floating_point_v<T>) return static_cast<T>(v.as_double());
        else if constexpr (std::is_same_v<T, std::string>) return v.as_string();
    }

    Json& operator[](const std::string& key) {
        if (!is_object()) value_ = Object{};
        return std::get<Object>(value_)[key];
    }

    void push_back(Json item) {
        if (!is_array()) value_ = Array{};
        std::get<Array>(value_).push_back(std::move(item));
    }

    std::string dump(int indent = -1, int current_indent = 0) const {
        std::ostringstream oss;
        dump_to(oss, indent, current_indent);
        return oss.str();
    }

    static Json parse(const std::string& text) {
        Parser p(text);
        Json res = p.parse_value();
        p.skip_ws();
        if (!p.eof()) {
            throw std::runtime_error("Unexpected trailing characters in JSON");
        }
        return res;
    }

    static Json parse_file(const std::string& path) {
        std::ifstream ifs(path);
        if (!ifs) {
            throw std::runtime_error("Failed to open JSON file: " + path);
        }
        std::ostringstream ss;
        ss << ifs.rdbuf();
        return parse(ss.str());
    }

private:
    Value value_;

    static void escape_string(std::ostringstream& oss, const std::string& s) {
        oss << '"';
        for (char c : s) {
            switch (c) {
                case '"':  oss << "\\\""; break;
                case '\\': oss << "\\\\"; break;
                case '\b': oss << "\\b";  break;
                case '\f': oss << "\\f";  break;
                case '\n': oss << "\\n";  break;
                case '\r': oss << "\\r";  break;
                case '\t': oss << "\\t";  break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<int>(static_cast<unsigned char>(c)) << std::dec;
                    } else {
                        oss << c;
                    }
            }
        }
        oss << '"';
    }

    void dump_to(std::ostringstream& oss, int indent, int depth) const {
        if (is_null()) {
            oss << "null";
        } else if (is_bool()) {
            oss << (as_bool() ? "true" : "false");
        } else if (is_int()) {
            oss << as_int();
        } else if (is_double()) {
            oss << std::setprecision(12) << as_double();
        } else if (is_string()) {
            escape_string(oss, as_string());
        } else if (is_array()) {
            const auto& arr = as_array();
            oss << '[';
            for (size_t i = 0; i < arr.size(); ++i) {
                if (i > 0) oss << ',';
                if (indent >= 0) oss << '\n' << std::string(static_cast<size_t>(depth + indent), ' ');
                arr[i].dump_to(oss, indent, depth + (indent >= 0 ? indent : 0));
            }
            if (!arr.empty() && indent >= 0) oss << '\n' << std::string(static_cast<size_t>(depth), ' ');
            oss << ']';
        } else if (is_object()) {
            const auto& obj = as_object();
            oss << '{';
            size_t i = 0;
            for (const auto& [k, v] : obj) {
                if (i++ > 0) oss << ',';
                if (indent >= 0) oss << '\n' << std::string(static_cast<size_t>(depth + indent), ' ');
                escape_string(oss, k);
                oss << (indent >= 0 ? ": " : ":");
                v.dump_to(oss, indent, depth + (indent >= 0 ? indent : 0));
            }
            if (!obj.empty() && indent >= 0) oss << '\n' << std::string(static_cast<size_t>(depth), ' ');
            oss << '}';
        }
    }

    class Parser {
    public:
        explicit Parser(const std::string& s) : s_(s), pos_(0) {}

        bool eof() const { return pos_ >= s_.size(); }

        void skip_ws() {
            while (!eof()) {
                char c = s_[pos_];
                if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                    ++pos_;
                } else if (c == '/' && pos_ + 1 < s_.size() && s_[pos_ + 1] == '/') {
                    while (!eof() && s_[pos_] != '\n') ++pos_;
                } else {
                    break;
                }
            }
        }

        Json parse_value() {
            skip_ws();
            if (eof()) throw std::runtime_error("Unexpected EOF in JSON");
            char c = s_[pos_];
            if (c == '{') return parse_object();
            if (c == '[') return parse_array();
            if (c == '"') return Json(parse_string());
            if (c == 't' || c == 'f') return parse_bool();
            if (c == 'n') return parse_null();
            if (c == '-' || (c >= '0' && c <= '9')) return parse_number();
            throw std::runtime_error(std::string("Unexpected JSON char: ") + c);
        }

    private:
        const std::string& s_;
        size_t pos_;

        char next() {
            if (eof()) throw std::runtime_error("Unexpected EOF in JSON");
            return s_[pos_++];
        }

        Json parse_object() {
            next(); // '{'
            Object obj;
            skip_ws();
            if (!eof() && s_[pos_] == '}') {
                ++pos_;
                return Json(std::move(obj));
            }
            while (true) {
                skip_ws();
                if (eof() || s_[pos_] != '"') throw std::runtime_error("Expected string key in JSON object");
                std::string key = parse_string();
                skip_ws();
                if (next() != ':') throw std::runtime_error("Expected ':' in JSON object");
                obj[std::move(key)] = parse_value();
                skip_ws();
                char c = next();
                if (c == '}') break;
                if (c != ',') throw std::runtime_error("Expected ',' or '}' in JSON object");
            }
            return Json(std::move(obj));
        }

        Json parse_array() {
            next(); // '['
            Array arr;
            skip_ws();
            if (!eof() && s_[pos_] == ']') {
                ++pos_;
                return Json(std::move(arr));
            }
            while (true) {
                arr.push_back(parse_value());
                skip_ws();
                char c = next();
                if (c == ']') break;
                if (c != ',') throw std::runtime_error("Expected ',' or ']' in JSON array");
            }
            return Json(std::move(arr));
        }

        std::string parse_string() {
            if (next() != '"') throw std::runtime_error("Expected '\"'");
            std::string out;
            while (!eof()) {
                char c = next();
                if (c == '"') return out;
                if (c == '\\') {
                    char e = next();
                    switch (e) {
                        case '"':  out.push_back('"'); break;
                        case '\\': out.push_back('\\'); break;
                        case '/':  out.push_back('/'); break;
                        case 'b':  out.push_back('\b'); break;
                        case 'f':  out.push_back('\f'); break;
                        case 'n':  out.push_back('\n'); break;
                        case 'r':  out.push_back('\r'); break;
                        case 't':  out.push_back('\t'); break;
                        default:   out.push_back(e); break;
                    }
                } else {
                    out.push_back(c);
                }
            }
            throw std::runtime_error("Unterminated JSON string");
        }

        Json parse_bool() {
            if (s_.compare(pos_, 4, "true") == 0) {
                pos_ += 4;
                return Json(true);
            }
            if (s_.compare(pos_, 5, "false") == 0) {
                pos_ += 5;
                return Json(false);
            }
            throw std::runtime_error("Invalid JSON boolean");
        }

        Json parse_null() {
            if (s_.compare(pos_, 4, "null") == 0) {
                pos_ += 4;
                return Json(nullptr);
            }
            throw std::runtime_error("Invalid JSON null");
        }

        Json parse_number() {
            size_t start = pos_;
            bool is_float = false;
            if (s_[pos_] == '-') ++pos_;
            while (!eof() && s_[pos_] >= '0' && s_[pos_] <= '9') ++pos_;
            if (!eof() && s_[pos_] == '.') {
                is_float = true;
                ++pos_;
                while (!eof() && s_[pos_] >= '0' && s_[pos_] <= '9') ++pos_;
            }
            if (!eof() && (s_[pos_] == 'e' || s_[pos_] == 'E')) {
                is_float = true;
                ++pos_;
                if (!eof() && (s_[pos_] == '+' || s_[pos_] == '-')) ++pos_;
                while (!eof() && s_[pos_] >= '0' && s_[pos_] <= '9') ++pos_;
            }
            std::string num_str = s_.substr(start, pos_ - start);
            if (is_float) {
                return Json(std::stod(num_str));
            }
            return Json(static_cast<int64_t>(std::stoll(num_str)));
        }
    };
};

} // namespace sim
