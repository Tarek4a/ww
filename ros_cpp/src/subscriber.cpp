#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std;
using namespace rclcpp;

class MsgSubscriber : public Node
{
public:
    MsgSubscriber() : Node("manual_sub")
    {
        sub_ = create_subscription<std_msgs::msg::String>(
            "letter_topic", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                cout << "Received: " << msg->data << endl;
            });
    }

private:
    Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    spin(make_shared<MsgSubscriber>());
    shutdown();
    return 0;
}
