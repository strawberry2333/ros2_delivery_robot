/**
 * @file test_delivery_manager_services.cpp
 * @brief DeliveryManager 服务级单元测试。
 *
 * 测试场景：
 * 1. SubmitOrder 成功入队
 * 2. SubmitOrder 重复 order_id 拒绝
 * 3. CancelOrder 成功/失败
 * 4. GetDeliveryReport 返回正确列表
 * 5. 优先级排序验证
 */

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "delivery_core/delivery_manager.hpp"
#include "delivery_interfaces/srv/cancel_order.hpp"
#include "delivery_interfaces/srv/get_delivery_report.hpp"
#include "delivery_interfaces/srv/submit_order.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DeliveryManagerServiceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

        // 获取测试用 stations.yaml 路径
    std::string test_dir = __FILE__;
    test_dir = test_dir.substr(0, test_dir.rfind('/'));
    stations_path_ = test_dir + "/test_stations.yaml";

        // 创建 manager 节点，使用测试配置
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"station_config", stations_path_},
      {"use_sim_time", false},
      {"publish_initial_pose", false},
        });
    manager_node_ = std::make_shared<delivery_core::DeliveryManager>(options);

        // 创建服务客户端的辅助节点
    client_node_ = rclcpp::Node::make_shared("test_client");
    submit_client_ = client_node_->create_client<delivery_interfaces::srv::SubmitOrder>(
            "submit_order");
    cancel_client_ = client_node_->create_client<delivery_interfaces::srv::CancelOrder>(
            "cancel_order");
    report_client_ = client_node_->create_client<delivery_interfaces::srv::GetDeliveryReport>(
            "get_delivery_report");

        // spin manager 节点的 executor
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(manager_node_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

        // 等待服务可用
    ASSERT_TRUE(submit_client_->wait_for_service(5s)) << "SubmitOrder 服务不可用";
    ASSERT_TRUE(cancel_client_->wait_for_service(5s)) << "CancelOrder 服务不可用";
    ASSERT_TRUE(report_client_->wait_for_service(5s)) << "GetDeliveryReport 服务不可用";
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    manager_node_.reset();
    client_node_.reset();
    rclcpp::shutdown();
  }

    // 辅助方法：提交订单
  delivery_interfaces::srv::SubmitOrder::Response::SharedPtr submit_order(
    const std::string & order_id,
    const std::string & pickup,
    const std::string & dropoff,
    uint8_t priority = 0)
  {
    auto request = std::make_shared<delivery_interfaces::srv::SubmitOrder::Request>();
    request->order.order_id = order_id;
    request->order.pickup_station = pickup;
    request->order.dropoff_station = dropoff;
    request->order.priority = priority;

    auto future = submit_client_->async_send_request(request);
    EXPECT_EQ(future.wait_for(5s), std::future_status::ready);
    return future.get();
  }

  std::string stations_path_;
  std::shared_ptr<delivery_core::DeliveryManager> manager_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<delivery_interfaces::srv::SubmitOrder>::SharedPtr submit_client_;
  rclcpp::Client<delivery_interfaces::srv::CancelOrder>::SharedPtr cancel_client_;
  rclcpp::Client<delivery_interfaces::srv::GetDeliveryReport>::SharedPtr report_client_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
};

TEST_F(DeliveryManagerServiceTest, SubmitOrderSuccess)
{
    auto response = submit_order("order_001", "station_A", "station_C");
    ASSERT_TRUE(response->accepted) << response->reason;
}

TEST_F(DeliveryManagerServiceTest, SubmitOrderDuplicateIdRejected)
{
    auto resp1 = submit_order("order_dup", "station_A", "station_C");
    ASSERT_TRUE(resp1->accepted);

    auto resp2 = submit_order("order_dup", "station_B", "station_C");
    ASSERT_FALSE(resp2->accepted);
    EXPECT_NE(resp2->reason.find("重复"), std::string::npos);
}

TEST_F(DeliveryManagerServiceTest, SubmitOrderInvalidStationRejected)
{
    auto response = submit_order("order_bad", "nonexistent", "station_C");
    ASSERT_FALSE(response->accepted);
}

TEST_F(DeliveryManagerServiceTest, CancelOrderSuccess)
{
    submit_order("order_cancel", "station_A", "station_C");

    auto request = std::make_shared<delivery_interfaces::srv::CancelOrder::Request>();
    request->order_id = "order_cancel";
    auto future = cancel_client_->async_send_request(request);
    ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
    auto response = future.get();
    EXPECT_TRUE(response->success) << response->reason;
}

TEST_F(DeliveryManagerServiceTest, CancelOrderNotFound)
{
    auto request = std::make_shared<delivery_interfaces::srv::CancelOrder::Request>();
    request->order_id = "nonexistent_order";
    auto future = cancel_client_->async_send_request(request);
    ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
    auto response = future.get();
    EXPECT_FALSE(response->success);
}

TEST_F(DeliveryManagerServiceTest, GetDeliveryReportReturnsQueuedOrders)
{
    submit_order("order_r1", "station_A", "station_C");
    submit_order("order_r2", "station_B", "station_C");

    auto request = std::make_shared<delivery_interfaces::srv::GetDeliveryReport::Request>();
    auto future = report_client_->async_send_request(request);
    ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
    auto response = future.get();
    ASSERT_GE(response->reports.size(), 2u);
}

TEST_F(DeliveryManagerServiceTest, PriorityOrdering)
{
    submit_order("order_low", "station_A", "station_C", 0);
    submit_order("order_high", "station_B", "station_C", 10);

    auto request = std::make_shared<delivery_interfaces::srv::GetDeliveryReport::Request>();
    auto future = report_client_->async_send_request(request);
    ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
    auto response = future.get();
    ASSERT_GE(response->reports.size(), 2u);
    // 高优先级订单应在前
    EXPECT_EQ(response->reports[0].order_id, "order_high");
    EXPECT_EQ(response->reports[1].order_id, "order_low");
}
