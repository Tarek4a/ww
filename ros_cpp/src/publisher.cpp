#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std;
using namespace rclcpp;

class MsgPublisher : public Node
{
public:
    MsgPublisher() : Node("pub")
    {
        pub_ = create_publisher<std_msgs::msg::String>("letter_topic", 10);
    }

    void loop()
    {
        while (ok())
        {
            string x;
            cout << "chose a أو b أو c: ";
            cin >> x;

            if (x != "a" && x != "b" && x != "c") {
                cout << "worng\n";
                continue;
            }

            std_msgs::msg::String msg;
            msg.data = x;
            pub_->publish(msg);

            RCLCPP_INFO(get_logger(), "Published: %s", msg.data.c_str());
        }
    }

private:
    Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    init(argc, argv);
    auto node = make_shared<MsgPublisher>();
    node->loop();
    shutdown();
    return 0;
}
