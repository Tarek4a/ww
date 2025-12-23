#include <memory>
#include <string>
#include <map>
#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class WaiterBotActionClient : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WaiterBotActionClient() : Node("waiter_bot_action_client") {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        
        // تعريف إحداثيات الـ 4 طاولات بناءً على طلبك (x, y, w)
        table_locations_[1] = {-3.0, 2.0, 1.0}; 
        table_locations_[2] = {3.0, 2.0, 1.0};  
        table_locations_[3] = {-3.0, -2.0, 1.0};
        table_locations_[4] = {3.0, -2.0, 1.0}; 

        // خيط (Thread) لانتظار إدخال المستخدم دون تعطيل البرنامج
        input_thread_ = std::thread(&WaiterBotActionClient::userInputLoop, this);
    }

    ~WaiterBotActionClient() {
        if (input_thread_.joinable()) input_thread_.join();
    }

private:
    struct Pose { double x, y, w; };
    std::map<int, Pose> table_locations_;
    std::thread input_thread_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

    void userInputLoop() {
        while (rclcpp::ok()) {
            int table_num;
            std::cout << "\n========================";
            std::cout << "\n   قائمة طاولات المطعم   ";
            std::cout << "\n========================";
            std::cout << "\n1. Table 1 (-3, 2)";
            std::cout << "\n2. Table 2 (3, 2)";
            std::cout << "\n3. Table 3 (-3, -2)";
            std::cout << "\n4. Table 4 (3, -2)";
            std::cout << "\nأدخل رقم الطاولة للذهاب إليها: ";
            
            if (!(std::cin >> table_num)) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                continue;
            }

            if (table_locations_.count(table_num)) {
                send_goal(table_locations_[table_num], table_num);
            } else {
                std::cout << "⚠️ رقم غير صحيح! اختر من 1 إلى 4.\n";
            }
        }
    }

    void send_goal(Pose p, int num) {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Navigation Server غير متاح!");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = p.x;
        goal_msg.pose.pose.position.y = p.y;
        goal_msg.pose.pose.orientation.w = p.w;

        std::cout << "🚀 الروبوت يتحرك الآن إلى طاولة رقم " << num << "...\n";

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.result_callback = [num](const GoalHandleNav::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                std::cout << "\n✅ تم الوصول بنجاح لطاولة رقم " << num << "!\n";
            } else {
                std::cout << "\n❌ فشل الوصول للطاولة رقم " << num << ".\n";
            }
        };

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaiterBotActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}