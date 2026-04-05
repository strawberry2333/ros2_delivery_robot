/**
 * @file test_delivery_manager_services.cpp
 * @brief DeliveryManager 服务接口单元测试。
 *
 * 这组测试把 DeliveryManager 当成一个 ROS 服务端来看待，
 * 重点验证接单、取消、查询报告以及站点角色校验等外部可观察行为。
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

/**
 * @brief DeliveryManager 服务测试夹具。
 *
 * 夹具会同时创建：
 * - 一个被测的 manager 服务端节点
 * - 一个专门发请求的 client 节点
 * - 一个负责推进回调的 executor 线程
 *
 * 这样每个测试都可以通过真实服务调用验证接口语义。
 */
class DeliveryManagerServiceTest : public ::testing::Test
{
protected:
  /**
   * @brief 初始化服务端、客户端和回调执行线程。
   */
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // 使用测试专用站点表，把输入合法性控制在可预测范围内。
    std::string test_dir = __FILE__;
    test_dir = test_dir.substr(0, test_dir.rfind('/'));
    stations_path_ = test_dir + "/test_stations.yaml";

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"station_config", stations_path_},
      {"use_sim_time", false},
      {"publish_initial_pose", false},
    });
    manager_node_ = std::make_shared<delivery_core::DeliveryManager>(options);

    // 这里手动置 ready，是为了把测试焦点锁定在服务行为，而不是系统启动链路。
    manager_node_->set_system_ready();

    // 单独创建一个调用方节点，模拟真实外部客户端。
    client_node_ = rclcpp::Node::make_shared("test_client");
    submit_client_ = client_node_->create_client<delivery_interfaces::srv::SubmitOrder>(
      "submit_order");
    cancel_client_ = client_node_->create_client<delivery_interfaces::srv::CancelOrder>(
      "cancel_order");
    report_client_ = client_node_->create_client<delivery_interfaces::srv::GetDeliveryReport>(
      "get_delivery_report");

    // 服务端和客户端都挂进同一个 executor，让请求和响应都能被正常推进。
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(manager_node_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    // 先确认服务已注册完成，避免把“服务不可见”误判成业务逻辑失败。
    ASSERT_TRUE(submit_client_->wait_for_service(5s)) << "SubmitOrder 服务不可用";
    ASSERT_TRUE(cancel_client_->wait_for_service(5s)) << "CancelOrder 服务不可用";
    ASSERT_TRUE(report_client_->wait_for_service(5s)) << "GetDeliveryReport 服务不可用";
  }

  /**
   * @brief 停止 executor 并回收测试资源。
   */
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

  /**
   * @brief 发送一条 SubmitOrder 请求并等待响应。
   * @param[in] order_id 订单 ID。
   * @param[in] pickup 取货站 ID。
   * @param[in] dropoff 送货站 ID。
   * @param[in] priority 优先级。
   * @return SubmitOrder 服务响应。
   */
  delivery_interfaces::srv::SubmitOrder::Response::SharedPtr submit_order(
    const std::string & order_id,
    const std::string & pickup,
    const std::string & dropoff,
    uint8_t priority = 0)
  {
    auto request = std::make_shared<delivery_interfaces::srv::SubmitOrder::Request>();

    // 把测试输入直接写进请求对象，后续每个用例只关心业务含义即可。
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

/**
 * @brief 验证合法订单会被接收。
 *
 * 这是 SubmitOrder 的最基本 happy path。
 */
TEST_F(DeliveryManagerServiceTest, SubmitOrderSuccess)
{
  auto response = submit_order("order_001", "station_A", "station_C");
  ASSERT_TRUE(response->accepted) << response->reason;
}

/**
 * @brief 验证重复的 order_id 会被拒绝。
 *
 * 这条测试防止系统把两张同名订单同时塞进队列。
 */
TEST_F(DeliveryManagerServiceTest, SubmitOrderDuplicateIdRejected)
{
  auto resp1 = submit_order("order_dup", "station_A", "station_C");
  ASSERT_TRUE(resp1->accepted);

  // 第二次复用相同 order_id，应触发去重保护。
  auto resp2 = submit_order("order_dup", "station_B", "station_C");
  ASSERT_FALSE(resp2->accepted);
  EXPECT_NE(resp2->reason.find("重复"), std::string::npos);
}

/**
 * @brief 验证不存在的站点不能通过接单校验。
 */
TEST_F(DeliveryManagerServiceTest, SubmitOrderInvalidStationRejected)
{
  auto response = submit_order("order_bad", "nonexistent", "station_C");
  ASSERT_FALSE(response->accepted);
}

/**
 * @brief 验证已入队订单可以被成功取消。
 */
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

/**
 * @brief 验证取消不存在的订单会返回失败。
 */
TEST_F(DeliveryManagerServiceTest, CancelOrderNotFound)
{
  auto request = std::make_shared<delivery_interfaces::srv::CancelOrder::Request>();
  request->order_id = "nonexistent_order";
  auto future = cancel_client_->async_send_request(request);

  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  auto response = future.get();
  EXPECT_FALSE(response->success);
}

/**
 * @brief 验证报告接口能返回当前队列中的订单。
 */
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

/**
 * @brief 验证报告中的订单顺序遵循优先级。
 */
TEST_F(DeliveryManagerServiceTest, PriorityOrdering)
{
  submit_order("order_low", "station_A", "station_C", 0);
  submit_order("order_high", "station_B", "station_C", 2);

  auto request = std::make_shared<delivery_interfaces::srv::GetDeliveryReport::Request>();
  auto future = report_client_->async_send_request(request);

  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  auto response = future.get();
  ASSERT_GE(response->reports.size(), 2u);

  // 高优先级订单应排在前面。
  EXPECT_EQ(response->reports[0].order_id, "order_high");
  EXPECT_EQ(response->reports[1].order_id, "order_low");
}

// ========== 站点类型白名单测试 ==========

/**
 * @brief 验证充电站不能被当成取货站。
 */
TEST_F(DeliveryManagerServiceTest, ChargeHomeAsPickupRejected)
{
  auto response = submit_order("order_charge_pickup", "charge_home", "station_C");
  ASSERT_FALSE(response->accepted);
  EXPECT_NE(response->reason.find("不是取货站"), std::string::npos) << response->reason;
}

/**
 * @brief 验证充电站不能被当成送货站。
 */
TEST_F(DeliveryManagerServiceTest, ChargeHomeAsDropoffRejected)
{
  auto response = submit_order("order_charge_dropoff", "station_A", "charge_home");
  ASSERT_FALSE(response->accepted);
  EXPECT_NE(response->reason.find("不是送货站"), std::string::npos) << response->reason;
}

/**
 * @brief 验证取货站类型不能被冒充成送货站。
 */
TEST_F(DeliveryManagerServiceTest, PickupStationAsDropoffRejected)
{
  auto response = submit_order("order_wrong_type", "station_A", "station_B");
  ASSERT_FALSE(response->accepted);
  EXPECT_NE(response->reason.find("不是送货站"), std::string::npos) << response->reason;
}
