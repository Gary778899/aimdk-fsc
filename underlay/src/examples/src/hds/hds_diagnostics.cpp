// HDS Diagnostics Tool — Subscribe to HDS topics or query history events.
//
// See docs/cn/example/Cpp.md section "Subscribe HDS diagnostics /
// query history events" for full usage and field descriptions.

#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "aimdk_msgs/msg/alert_code_array.hpp"
#include "aimdk_msgs/msg/diagnostic_info_array.hpp"
#include "aimdk_msgs/msg/diagnostic_type.hpp"
#include "aimdk_msgs/msg/hds_event_query.hpp"
#include "aimdk_msgs/msg/post_proc_sig_array.hpp"
#include "aimdk_msgs/srv/get_hds_event_history.hpp"

using namespace std::chrono_literals;

namespace {

std::string DiagTypeName(int32_t value) {
  switch (value) {
  case aimdk_msgs::msg::DiagnosticType::NORMAL:
    return "NORMAL";
  case aimdk_msgs::msg::DiagnosticType::TRIGGER_APPEAR:
    return "APPEAR";
  case aimdk_msgs::msg::DiagnosticType::TRIGGER_DISAPPEAR:
    return "DISAPPEAR";
  default:
    return "UNKNOWN(" + std::to_string(value) + ")";
  }
}

std::string StateName(uint8_t value) {
  switch (value) {
  case aimdk_msgs::msg::HdsEventQuery::STATE_ALL:
    return "ALL";
  case aimdk_msgs::msg::HdsEventQuery::STATE_ACTIVE:
    return "ACTIVE";
  case aimdk_msgs::msg::HdsEventQuery::STATE_RECOVERED:
    return "RECOVERED";
  default:
    return "UNKNOWN";
  }
}

std::string HexCode(uint64_t code, int width = 10) {
  std::ostringstream oss;
  oss << "0x" << std::hex << std::uppercase << std::setw(width)
      << std::setfill('0') << code;
  return oss.str();
}

uint64_t ParseHex(const std::string &s) { return std::stoull(s, nullptr, 0); }

// ---------- echo mode ----------

class HdsDiagnosticsEcho : public rclcpp::Node {
public:
  HdsDiagnosticsEcho() : Node("hds_diagnostics_echo") {
    // HDS publishers use BEST_EFFORT; match it to receive messages.
    auto qos = rclcpp::QoS(10).best_effort().durability_volatile();

    sub_diag_ = create_subscription<aimdk_msgs::msg::DiagnosticInfoArray>(
        "/aima/hds/diag_code_list", qos,
        std::bind(&HdsDiagnosticsEcho::OnDiagnostics, this,
                  std::placeholders::_1));

    sub_alert_ = create_subscription<aimdk_msgs::msg::AlertCodeArray>(
        "/aima/hds/alert_code_list", qos,
        std::bind(&HdsDiagnosticsEcho::OnAlerts, this, std::placeholders::_1));

    sub_post_ = create_subscription<aimdk_msgs::msg::PostProcSigArray>(
        "/aima/hds/post_proc/signal", qos,
        std::bind(&HdsDiagnosticsEcho::OnPostProc, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed: diag_code_list / alert_code_list / "
                              "post_proc/signal");
  }

private:
  void
  OnDiagnostics(const aimdk_msgs::msg::DiagnosticInfoArray::SharedPtr msg) {
    if (msg->diagnostics.empty())
      return;
    std::ostringstream oss;
    oss << "[Diagnostics] count=" << msg->diagnostics.size();
    for (const auto &d : msg->diagnostics) {
      oss << "\n  diag_code=" << HexCode(d.diag_code)
          << "  type=" << DiagTypeName(d.type.value)
          << "  module_id=" << static_cast<int>(d.module_id)
          << "  ts=" << d.timestamp << "  info=\"" << d.info << "\"";
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  void OnAlerts(const aimdk_msgs::msg::AlertCodeArray::SharedPtr msg) {
    if (msg->list.empty())
      return;
    std::ostringstream oss;
    oss << "[Alerts] count=" << msg->list.size();
    for (const auto &a : msg->list) {
      oss << "\n  code=\"" << a.code << "\"  ts=" << a.timestamp
          << "  first_ts=" << a.first_timestamp;
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  void OnPostProc(const aimdk_msgs::msg::PostProcSigArray::SharedPtr msg) {
    if (msg->post_proc_sig.empty())
      return;
    std::ostringstream oss;
    oss << "[PostProc] count=" << msg->post_proc_sig.size();
    for (const auto &p : msg->post_proc_sig) {
      const auto &s = p.sig_set;
      oss << "\n  diag_code=" << HexCode(p.diag_code) << "  ts=" << p.timestamp
          << "  robot=" << s.robot_sig
          << " motion=" << static_cast<int>(s.motion_sig)
          << " sound=" << static_cast<int>(s.sound_sig)
          << " led=" << static_cast<int>(s.led_sig)
          << " facial=" << static_cast<int>(s.facial_sig)
          << " funcdisable=" << HexCode(s.funcdisable_sig, 16);
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  rclcpp::Subscription<aimdk_msgs::msg::DiagnosticInfoArray>::SharedPtr
      sub_diag_;
  rclcpp::Subscription<aimdk_msgs::msg::AlertCodeArray>::SharedPtr sub_alert_;
  rclcpp::Subscription<aimdk_msgs::msg::PostProcSigArray>::SharedPtr sub_post_;
};

int RunEcho(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HdsDiagnosticsEcho>());
  rclcpp::shutdown();
  return 0;
}

// ---------- history mode ----------

struct HistoryArgs {
  uint64_t start_ns = 0;
  uint64_t end_ns = 0;
  std::vector<uint64_t> diag_codes;
  std::vector<std::string> alert_codes;
  uint8_t state = aimdk_msgs::msg::HdsEventQuery::STATE_ALL;
  bool desc = false;
  uint32_t offset = 0;
  uint32_t limit = 20;
};

bool ParseHistoryArgs(int argc, char **argv, HistoryArgs *args) {
  // argv[0] = exe, argv[1] = "history", real options start at 2.
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    auto need_value = [&](const char *flag) -> const char * {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << flag << std::endl;
        return nullptr;
      }
      return argv[++i];
    };

    if (a == "--start-ns") {
      const char *v = need_value("--start-ns");
      if (!v)
        return false;
      args->start_ns = std::stoull(v);
    } else if (a == "--end-ns") {
      const char *v = need_value("--end-ns");
      if (!v)
        return false;
      args->end_ns = std::stoull(v);
    } else if (a == "--state") {
      const char *v = need_value("--state");
      if (!v)
        return false;
      args->state = static_cast<uint8_t>(std::stoi(v));
    } else if (a == "--offset") {
      const char *v = need_value("--offset");
      if (!v)
        return false;
      args->offset = static_cast<uint32_t>(std::stoul(v));
    } else if (a == "--limit") {
      const char *v = need_value("--limit");
      if (!v)
        return false;
      args->limit = static_cast<uint32_t>(std::stoul(v));
    } else if (a == "--desc") {
      args->desc = true;
    } else if (a == "--diag-codes") {
      while (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
        args->diag_codes.push_back(ParseHex(argv[++i]));
      }
    } else if (a == "--alert-codes") {
      while (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
        args->alert_codes.emplace_back(argv[++i]);
      }
    } else {
      std::cerr << "Unknown option: " << a << std::endl;
      return false;
    }
  }
  return true;
}

void PrintRecords(
    const aimdk_msgs::srv::GetHdsEventHistory::Response &response) {
  std::cout << "total_count = " << response.total_count
            << ", returned = " << response.records.size()
            << ", has_more = " << (response.has_more ? "true" : "false")
            << std::endl;
  if (response.records.empty())
    return;

  std::string sep(80, '-');
  std::cout << sep << std::endl;
  for (const auto &r : response.records) {
    double duration_s = static_cast<double>(r.duration_ns) / 1e9;
    std::cout << "id=" << r.id << "  diag=" << HexCode(r.diag_code)
              << "  alert=\"" << r.alert_code << "\"" << std::endl;
    std::cout << "  first_ts=" << r.first_timestamp
              << "  last_ts=" << r.last_timestamp << std::endl;
    std::cout << "  recovered=" << (r.recovered ? "YES" : "NO")
              << "  recover_ts=" << r.recover_timestamp
              << "  duration=" << std::fixed << std::setprecision(3)
              << duration_s << "s" << std::endl;
    if (!r.alert_location.empty()) {
      std::cout << "  location=\"" << r.alert_location << "\"" << std::endl;
    }
    std::cout << sep << std::endl;
  }
}

int RunHistory(int argc, char **argv) {
  HistoryArgs args;
  if (!ParseHistoryArgs(argc, argv, &args))
    return 1;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hds_history_client");
  auto client = node->create_client<aimdk_msgs::srv::GetHdsEventHistory>(
      "/aimdk_5Fmsgs/srv/GetHdsEventHistory");

  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(),
                 "Service /aimdk_5Fmsgs/srv/GetHdsEventHistory not available");
    rclcpp::shutdown();
    return 1;
  }

  auto request =
      std::make_shared<aimdk_msgs::srv::GetHdsEventHistory::Request>();
  auto now = node->now();
  request->header.stamp.sec = now.seconds();
  request->header.stamp.nanosec = now.nanoseconds() % 1000000000;
  request->query.start_time = args.start_ns;
  request->query.end_time = args.end_ns;
  request->query.diag_codes = args.diag_codes;
  request->query.alert_codes = args.alert_codes;
  request->query.state = args.state;
  request->query.order_desc = args.desc;
  request->query.offset = args.offset;
  request->query.limit = args.limit;

  RCLCPP_INFO(node->get_logger(),
              "Querying history: state=%s, offset=%u, limit=%u, desc=%s",
              StateName(args.state).c_str(), args.offset, args.limit,
              args.desc ? "true" : "false");

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Service call timeout");
    rclcpp::shutdown();
    return 1;
  }

  PrintRecords(*future.get());
  rclcpp::shutdown();
  return 0;
}

void PrintUsage(const char *prog) {
  std::cout << "Usage:\n"
            << "  " << prog << " echo\n"
            << "  " << prog
            << " history [--state N] [--limit N] [--offset N] [--desc]\n"
            << "                  [--start-ns N] [--end-ns N]\n"
            << "                  [--diag-codes 0xAAAA 0xBBBB ...]\n"
            << "                  [--alert-codes \"code1\" \"code2\" ...]\n";
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  std::string mode = argv[1];
  if (mode == "echo")
    return RunEcho(argc, argv);
  if (mode == "history")
    return RunHistory(argc, argv);

  PrintUsage(argv[0]);
  return 1;
}
