#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <tello_msgs/msg/tello_position.hpp>

namespace tello_gazebo {
    class TelloPositionSyncPlugin : public gazebo::ModelPlugin {
        // GazeboROS node
        gazebo_ros::Node::SharedPtr node_;

        // ROS subscriptions
        rclcpp::Subscription<tello_msgs::msg::TelloPosition>::SharedPtr position_sub_;

        // Pointer to the model
        gazebo::physics::ModelPtr model_;

        // Pointer to the link
        gazebo::physics::LinkPtr base_link_;

        // Last received position
        tello_msgs::msg::TelloPosition::SharedPtr last_position_;

    public:
        TelloPositionSyncPlugin() {}
        ~TelloPositionSyncPlugin() {}

        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
            GZ_ASSERT(model != nullptr, "Model is null");
            GZ_ASSERT(sdf != nullptr, "SDF is null");

            std::string link_name{"base_link"};

            // In theory we can move much of this config into the <ros> tag, perhaps it's finished in Eloquent?
            if (sdf->HasElement("link_name")) {
                link_name = sdf->GetElement("link_name")->Get<std::string>();
            }

            base_link_ = model->GetLink(link_name);
            GZ_ASSERT(base_link_ != nullptr, "Missing link");

            std::cout << std::fixed;
            std::setprecision(2);
            std::cout << std::endl;
            std::cout << "TELLO PLUGIN" << std::endl;
            std::cout << "-----------------------------------------" << std::endl;
            std::cout << "link_name: " << link_name << std::endl;
            std::cout << "-----------------------------------------" << std::endl;
            std::cout << std::endl;

            // ROS node
            node_ = gazebo_ros::Node::Get(sdf);

            // Fix by adding <parameter name="use_sim_time" type="bool">1</parameter> to the SDF file
            bool use_sim_time;
            node_->get_parameter("use_sim_time", use_sim_time);
            if (!use_sim_time) {
                RCLCPP_ERROR(node_->get_logger(), "use_sim_time is false, could be a bug");
            }

            position_sub_ = node_->create_subscription<tello_msgs::msg::TelloPosition>(
                "tello_position",
                10,
                std::bind(
                    &TelloPositionSyncPlugin::OnPositionUpdate,
                    this,
                    std::placeholders::_1
            ));

            update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(
                    &TelloPositionSyncPlugin::OnUpdate,
                    this
            ));
        }

        void OnPositionUpdate(const tello_msgs::msg::TelloPosition::SharedPtr msg) {
            last_position_ = msg;
        }

        void OnUpdate() {
            if (last_position_) {
                ignition::math::Pose3d pose(
                    ignition::math::Vector3d(5*(last_position_->x), 5*(last_position_->y), last_position_->z),
                    ignition::math::Quaterniond(3*(last_position_->roll* M_PI / 180.0), 3*(last_position_->pitch* M_PI / 180.0), last_position_->yaw* M_PI / 180.0)
                );
                base_link_->SetWorldPose(pose);
            }
        }

    private:
        gazebo::event::ConnectionPtr update_connection_;
    };

    GZ_REGISTER_MODEL_PLUGIN(TelloPositionSyncPlugin)
} // namespace tello_gazebo
