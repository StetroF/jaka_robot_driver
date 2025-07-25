#include "rclcpp/rclcpp.hpp"
#include "JAKAZuRobot.h"
#include "jaka_robot_interfaces/srv/multi_mov_j.hpp"
#include "jaka_robot_interfaces/srv/multi_mov_l.hpp"
#include "jaka_robot_interfaces/srv/servo_mov_j.hpp"
#include "jaka_robot_interfaces/srv/joint_to_cartesian.hpp"
#include "jaka_robot_interfaces/msg/joint_value.hpp"
#include "jaka_robot_interfaces/msg/cartesian_pose.hpp"
#include "jaka_robot_interfaces/msg/robot_state_dual.hpp"
#include "jaka_robot_interfaces/msg/servo_joint_command.hpp"
#include "jaka_robot_interfaces/msg/servo_cart_command.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <thread>
#include <mutex>
#include <cmath>
#include <atomic>
#include <deque>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "jaka_robot_driver/jkerr_code.h"
#define deg2rad(x) ((x) * M_PI / 180.0)
#ifndef AXIS_NUM
#define AXIS_NUM 7
#endif

using std::placeholders::_1;
using ServoJointCommand = jaka_robot_interfaces::msg::ServoJointCommand;



class JakaDriverNode : public rclcpp::Node
{
public:
    JakaDriverNode() : Node("jaka_driver_node")
    {
        // 初始化参数

        // 所有参数在开头声明
        this->declare_parameter<bool>("servo_mode_enable", false); // 1:开, 0:关
        this->declare_parameter<double>("servo_joint_vel", 360.0);
        this->declare_parameter<double>("servo_joint_acc", 240.0);
        this->declare_parameter<double>("servo_joint_jerk", 720.0);
        this->declare_parameter<bool>("servo_cart_test_mode", false);


        current_servo_vel_ = this->get_parameter("servo_joint_vel").as_double();
        current_servo_acc_ = this->get_parameter("servo_joint_acc").as_double();
        current_servo_jerk_ = this->get_parameter("servo_joint_jerk").as_double();
        RCLCPP_INFO(this->get_logger(), "伺服速度: %.3f, 伺服加速度: %.3f, 伺服加加速度: %.3f", current_servo_vel_, current_servo_acc_, current_servo_jerk_);
        
        robot_ = std::make_shared<JAKAZuRobot>();


        int ret = robot_->login_in("192.168.2.200"); // 示例 IP
        robot_->servo_move_use_none_filter();
        robot_->motion_abort();
        robot_->servo_move_enable(0, -1);


        if (ret == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Connected to robot!.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot. Code: %d", ret);
        }

        // 上电机器人
        ret = robot_->power_on();
        if (ret != ERR_SUCC)
        {
            RCLCPP_ERROR(this->get_logger(), "Power on failed. Code: %d", ret);
            robot_->login_out(); // 断开连接
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Robot powered on.");

        // 使能机器人
        ret = robot_->enable_robot();

  
        if (ret != ERR_SUCC)
        {
            RCLCPP_ERROR(this->get_logger(), "Enable robot failed. Code: %d", ret);
            robot_->power_off();
            robot_->login_out();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Robot enabled.");

        // 获取机器人状态
        RobotState state;
        ret = robot_->get_robot_state(&state);
        if (!state.servoEnabled)
        {
            ErrorCode code;
            robot_->get_last_error(&code);
            RCLCPP_ERROR(this->get_logger(), "Robot is error! error code = %2x, msg = %s", code.code, code.message);
            robot_->power_off();
            return;
        }



        // 设置碰撞检测级别
        robot_->set_collision_level(LEFT, 4);  // 设置左臂碰撞级别
        robot_->set_collision_level(RIGHT, 4); // 设置右臂碰撞级别
        RCLCPP_INFO(this->get_logger(), "Collision level set to 4.");


        
        bool servo_joint = this->get_parameter("servo_mode_enable").as_bool();
        RCLCPP_INFO(this->get_logger(), "伺服模式: %d", servo_joint);
        if (servo_joint)
        {
            JointValue init_pos[2];
            memset(&init_pos, 0, sizeof(init_pos));
            //
            double v[] = {deg2rad(30), deg2rad(30)};
            double a[] = {deg2rad(150), deg2rad(150)};
            MoveMode mode[] = {MoveMode::ABS, MoveMode::ABS};
            robot_->robot_run_multi_movj(DUAL, mode, true, init_pos, v, a);
            RCLCPP_INFO(this->get_logger(), "伺服模式下回到初始位置.");

            servo_thread_ = std::thread(&JakaDriverNode::servo_loop, this);
        }



        bool servo_cart = this->get_parameter("servo_cart_test_mode").as_bool();
        if (servo_cart)
        {
            // Step 1: 获取目标笛卡尔位姿
            CartesianPose cpos[1]; // 只关注左臂
            JointValue jpos[1];    // 只关注左臂
            memset(&cpos, 0, sizeof(cpos));
            memset(&jpos, 0, sizeof(jpos));

            // 设置左臂关节角度（使用你提供的数据）
            jpos[0].jVal[0] = deg2rad(-74);
            jpos[0].jVal[1] = deg2rad(-5);
            jpos[0].jVal[2] = deg2rad(67);
            jpos[0].jVal[3] = deg2rad(-51);
            jpos[0].jVal[4] = deg2rad(2);
            jpos[0].jVal[5] = deg2rad(-40);
            jpos[0].jVal[6] = deg2rad(-20);

            memcpy(&jpos[1], &jpos[0], sizeof(jpos[0]));

            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                // 左臂目标位姿
                cpos[0].tran.x = pose_cmd_.end_pose_left.x;
                cpos[0].tran.y = pose_cmd_.end_pose_left.y;
                cpos[0].tran.z = pose_cmd_.end_pose_left.z;
                cpos[0].rpy.rx = pose_cmd_.end_pose_left.rx;
                cpos[0].rpy.ry = pose_cmd_.end_pose_left.ry;
                cpos[0].rpy.rz = pose_cmd_.end_pose_left.rz;
            }

            // Step 2: 进行逆解，获得关节位姿
            int ret_l = robot_->kine_inverse(0, nullptr, &cpos[0], &jpos[0]);

            if (ret_l != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed: left=%d", ret_l);
            }
            else
            {
                double v[] = {deg2rad(10)};
                double a[] = {deg2rad(50)};
                MoveMode mode[] = {MoveMode::ABS};

                // Step 3: 控制机器人先走到目标 pose 的位置（通过关节方式）
                robot_->robot_run_multi_movj(LEFT, mode, true, jpos, v, a);
                RCLCPP_INFO(this->get_logger(), "Moved to Cartesian target for servo_p test.");
            }

            // Step 4: 启动伺服线程
            servo_pose_thread_ = std::thread(&JakaDriverNode::servo_loop_pose_left, this);
        }

        // ROS2 接口
        srv_mov_j = this->create_service<jaka_robot_interfaces::srv::MultiMovJ>(
            "multi_movj", std::bind(&JakaDriverNode::handle_multi_movj, this, std::placeholders::_1, std::placeholders::_2));

        srv_mov_l = this->create_service<jaka_robot_interfaces::srv::MultiMovL>(
            "multi_movl", std::bind(&JakaDriverNode::handle_multi_movl, this, std::placeholders::_1, std::placeholders::_2));

        srv_joint_to_cartesian_ = this->create_service<jaka_robot_interfaces::srv::JointToCartesian>(
            "joint_to_cartesian", std::bind(&JakaDriverNode::handle_joint_to_cartesian, this, std::placeholders::_1, std::placeholders::_2));

        publisher_ = this->create_publisher<jaka_robot_interfaces::msg::RobotStateDual>("robot_state_dual", 10);
        
        clear_error_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "clear_error", std::bind(&JakaDriverNode::handle_clear_error, this, std::placeholders::_1, std::placeholders::_2));
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JakaDriverNode::timer_callback, this));

        servo_sub_ = this->create_subscription<jaka_robot_interfaces::msg::ServoJointCommand>(
            "/servo_joint_command", 10,
            std::bind(&JakaDriverNode::servo_callback, this, std::placeholders::_1));
        cartesian_cmd_sub_ = this->create_subscription<jaka_robot_interfaces::msg::ServoCartCommand>(
            "/servo_cart_command", 10,
            std::bind(&JakaDriverNode::servo_cart_command_callback, this, std::placeholders::_1));


        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&JakaDriverNode::dynamic_parameter_callback, this, std::placeholders::_1));



        // 启动后台错误检测线程
        error_check_thread_ = std::thread(&JakaDriverNode::error_monitor_loop, this);
        RCLCPP_INFO(this->get_logger(), "Monitor thread started.");
        RCLCPP_INFO(this->get_logger(), "JAKA ROS2 driver node started.");

        robot_->set_error_handler(JakaDriverNode::robot_error_callback);
        // robot_->login_out();
        // RCLCPP_INFO(this->get_logger(), "Logged out from robot.");
        // robot_->clear_error();
    }

public:
    ~JakaDriverNode()
    {
        running_joint_ = false;
        if (servo_thread_.joinable())
            servo_thread_.join();
        running_pose_ = false;
        if (servo_pose_thread_.joinable())
            servo_pose_thread_.join();

        if (error_check_thread_.joinable())
        {
            error_check_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "JakaDriverNode shutting down.");
    }
    void check_return(int ret, const std::string& func_name) {
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "%s failed, return value: %d", func_name.c_str(), ret);
        }
    }
private:

    rcl_interfaces::msg::SetParametersResult dynamic_parameter_callback(const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "ok";
        for (const auto &param : params)
        {
            if (param.get_name() == "servo_mode_enable")
            {
                bool mode = param.as_bool();
                if (mode != current_servo_mode_)
                {
                    int ret = robot_->servo_move_enable(mode, -1);
                    if (ret == 0)
                    {
                        current_servo_mode_ = mode;
                        RCLCPP_INFO(this->get_logger(), "[动态参数] 伺服模式已%s", mode ? "开启" : "关闭");
                    }
                    else
                    {
                        result.successful = false;
                        result.reason = "servo_move_enable调用失败";
                        RCLCPP_ERROR(this->get_logger(), "servo_move_enable(%d, -1) 失败, ret=%d", mode, ret);
                    }
                }
            }
            if (param.get_name() == "servo_joint_vel")
            {
                double v = param.as_double();
                if (current_servo_mode_ == 1)
                {
                    RCLCPP_WARN(this->get_logger(), "[动态参数] 伺服模式下不能设置速度参数，请先关闭伺服模式");
                    result.successful = false;
                    result.reason = "伺服模式下不能设置速度";
                }
                else
                {
                    int ret = robot_->servo_move_use_joint_NLF(v, current_servo_acc_, current_servo_jerk_);
                    if (ret == 0)
                    {
                        current_servo_vel_ = v;
                        RCLCPP_INFO(this->get_logger(), "[动态参数] 伺服速度已设置为%.3f", v);
                    }
                    else
                    {
                        result.successful = false;
                        result.reason = "servo_move_use_joint_NLF调用失败";
                        RCLCPP_ERROR(this->get_logger(), "servo_move_use_joint_NLF(%.3f, %.3f, %.3f) 失败, ret=%d", v, current_servo_acc_, current_servo_jerk_, ret);
                    }
                }
            }
            if (param.get_name() == "servo_joint_acc")
            {
                double a = param.as_double();
                if (current_servo_mode_ == 1)
                {
                    RCLCPP_WARN(this->get_logger(), "[动态参数] 伺服模式下不能设置加速度参数，请先关闭伺服模式");
                    result.successful = false;
                    result.reason = "伺服模式下不能设置加速度";
                }
                else
                {
                    int ret = robot_->servo_move_use_joint_NLF(current_servo_vel_, a, current_servo_jerk_);
                    if (ret == 0)
                    {
                        current_servo_acc_ = a;
                        RCLCPP_INFO(this->get_logger(), "[动态参数] 伺服加速度已设置为%.3f", a);
                    }
                    else
                    {
                        result.successful = false;
                        result.reason = "servo_move_use_joint_NLF调用失败";
                        RCLCPP_ERROR(this->get_logger(), "servo_move_use_joint_NLF(%.3f, %.3f, %.3f) 失败, ret=%d", current_servo_vel_, a, current_servo_jerk_, ret);
                    }
                }
            }
            if (param.get_name() == "servo_joint_jerk")
            {
                double j = param.as_double();
                if (current_servo_mode_ == 1)
                {
                    RCLCPP_WARN(this->get_logger(), "[动态参数] 伺服模式下不能设置加加速度参数，请先关闭伺服模式");
                    result.successful = false;
                    result.reason = "伺服模式下不能设置加加速度";
                }
                else
                {
                    int ret = robot_->servo_move_use_joint_NLF(current_servo_vel_, current_servo_acc_, j);
                    if (ret == 0)
                    {
                        current_servo_jerk_ = j;
                        RCLCPP_INFO(this->get_logger(), "[动态参数] 伺服加加速度已设置为%.3f", j);
                    }
                    else
                    {
                        result.successful = false;
                        result.reason = "servo_move_use_joint_NLF调用失败";
                        RCLCPP_ERROR(this->get_logger(), "servo_move_use_joint_NLF(%.3f, %.3f, %.3f) 失败, ret=%d", current_servo_vel_, current_servo_acc_, j, ret);
                    }
                }
            }
        }
        return result;
    }
    void handle_clear_error(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        robot_->clear_error();
        RCLCPP_INFO(this->get_logger(), "Error cleared.");
        response->success = true;
    }

    void handle_multi_movj(
        const std::shared_ptr<jaka_robot_interfaces::srv::MultiMovJ::Request> request,
        std::shared_ptr<jaka_robot_interfaces::srv::MultiMovJ::Response> response)
    {
        BOOL is_block = request->is_block ? TRUE : FALSE;

        MoveMode left_move_mode = static_cast<MoveMode>(request->left_move_mode.mode);
        MoveMode right_move_mode = static_cast<MoveMode>(request->right_move_mode.mode);

        MoveMode move_modes[2] = {left_move_mode, right_move_mode};

        JointValue joint_pos[2];

        for (int i = 0; i < 7; ++i)
        {
            joint_pos[0].jVal[i] = request->joint_pos_left.joint_values[i];  // 左臂关节
            joint_pos[1].jVal[i] = request->joint_pos_right.joint_values[i]; // 右臂关节
        }

        double vel[2] = {request->vel[0], request->vel[1]}; // 速度
        double acc[2] = {request->acc[0], request->acc[1]}; // 加速度

        errno_t ret;
        // 调用 SDK 函数
        ret = robot_->robot_run_multi_movj(request->robot_id, move_modes, is_block, joint_pos, vel, acc);
        if (ret == 0)
        {
            response->ret_code = 0;
            response->message = "Success";
            RCLCPP_INFO(this->get_logger(), "Multi-joint motion executed successfully.");
        }
        else
        {
            response->ret_code = -1;
            response->message = "Failed to execute multi-joint motion";
            RCLCPP_ERROR(this->get_logger(), "Error while executing multi-joint motion");
        }
    }

    void handle_multi_movl(
        const std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL::Request> request,
        std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL::Response> response)
    {
        BOOL is_block = request->is_block ? TRUE : FALSE;

        // 提取左右臂的运动模式
        MoveMode left_move_mode = static_cast<MoveMode>(request->left_move_mode.mode);
        MoveMode right_move_mode = static_cast<MoveMode>(request->right_move_mode.mode);
        MoveMode move_modes[2] = {left_move_mode, right_move_mode};

        // 提取左右臂笛卡尔目标位姿
        CartesianPose end_pos[2];
        end_pos[0].tran.x = request->end_pos_left.x;
        end_pos[0].tran.y = request->end_pos_left.y;
        end_pos[0].tran.z = request->end_pos_left.z;
        end_pos[0].rpy.rx = request->end_pos_left.rx;
        end_pos[0].rpy.ry = request->end_pos_left.ry;
        end_pos[0].rpy.rz = request->end_pos_left.rz;

        end_pos[1].tran.x = request->end_pos_right.x;
        end_pos[1].tran.y = request->end_pos_right.y;
        end_pos[1].tran.z = request->end_pos_right.z;
        end_pos[1].rpy.rx = request->end_pos_right.rx;
        end_pos[1].rpy.ry = request->end_pos_right.ry;
        end_pos[1].rpy.rz = request->end_pos_right.rz;

        // 提取速度和加速度
        double vel[2] = {request->vel[0], request->vel[1]};
        double acc[2] = {request->acc[0], request->acc[1]};

        // 调用 SDK 函数
        errno_t ret = robot_->robot_run_multi_movl(request->robot_id, move_modes, is_block, end_pos, vel, acc);
        if (ret == 0)
        {
            response->ret_code = 0;
            response->message = "Success";
            RCLCPP_INFO(this->get_logger(), "Multi-linear motion executed successfully.");
        }
        else
        {
            response->ret_code = -1;
            response->message = "Failed to execute multi-linear motion";
            RCLCPP_ERROR(this->get_logger(), "Error while executing multi-linear motion");
        }
    }

    void handle_joint_to_cartesian(
        const std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian::Request> request,
        std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian::Response> response)
    {
        // 验证机器人ID
        if (request->robot_id > 1)
        {
            response->success = false;
            response->message = "Invalid robot_id. Must be 0 (left arm) or 1 (right arm).";
            RCLCPP_ERROR(this->get_logger(), "Invalid robot_id: %d", request->robot_id);
            return;
        }

        // 将ROS消息转换为SDK结构
        JointValue joint_pos;
        memset(&joint_pos, 0, sizeof(joint_pos));
        
        for (int i = 0; i < 7; ++i)
        {
            joint_pos.jVal[i] = request->joint_value.joint_values[i];
        }

        // 调用正运动学函数
        CartesianPose cartesian_pose;
        memset(&cartesian_pose, 0, sizeof(cartesian_pose));
        
        errno_t ret = robot_->kine_forward(request->robot_id, &joint_pos, &cartesian_pose);
        
        if (ret == 0)
        {
            // 转换成功，填充响应
            response->cartesian_pose.x = cartesian_pose.tran.x;
            response->cartesian_pose.y = cartesian_pose.tran.y;
            response->cartesian_pose.z = cartesian_pose.tran.z;
            response->cartesian_pose.rx = cartesian_pose.rpy.rx;
            response->cartesian_pose.ry = cartesian_pose.rpy.ry;
            response->cartesian_pose.rz = cartesian_pose.rpy.rz;
            
            response->success = true;
            response->message = "Forward kinematics calculation successful";
            
            RCLCPP_INFO(this->get_logger(), "Forward kinematics for robot %d: pos=(%.3f, %.3f, %.3f), rot=(%.3f, %.3f, %.3f)",
                        request->robot_id,
                        response->cartesian_pose.x, response->cartesian_pose.y, response->cartesian_pose.z,
                        response->cartesian_pose.rx, response->cartesian_pose.ry, response->cartesian_pose.rz);
        }
        else
        {
            response->success = false;
            response->message = "Forward kinematics calculation failed";
            RCLCPP_ERROR(this->get_logger(), "Forward kinematics failed for robot %d, error code: %d", request->robot_id, ret);
        }
    }

    void timer_callback()
    {
        if (!robot_)
        {
            RCLCPP_ERROR(this->get_logger(), "robot_ is nullptr!");
            return;
        }
        struct timespec ts;
        errno_t ret = robot_->edg_recv(&ts); // 传一个有效指针
        // errno_t ret = robot_->edg_recv(nullptr);
        if (ret != 0)
        {
            RCLCPP_WARN(this->get_logger(), "edg_recv failed: %d", ret);
            return;
        }

        jaka_robot_interfaces::msg::RobotStateDual msg;

        for (int robot_index = 0; robot_index <= 1; ++robot_index)
        {
            JointValue joint_pos;
            CartesianPose cartesian_pose;

            ret = robot_->edg_get_stat(robot_index, &joint_pos, &cartesian_pose);
            auto &target = (robot_index == 0) ? jpos_left_ : jpos_right_;
            target = joint_pos;
            
            if (ret != 0)
            {
                RCLCPP_WARN(this->get_logger(), "edg_get_stat failed for arm %d: %d", robot_index, ret);
                continue;
            }

            auto &joint_msg = (robot_index == 0) ? msg.joint_pos_left : msg.joint_pos_right;
            auto &pose_msg = (robot_index == 0) ? msg.end_pose_left : msg.end_pose_right;

            for (int i = 0; i < 7; ++i)
            {
                joint_msg.joint_values[i] = joint_pos.jVal[i];
            }

            pose_msg.x = cartesian_pose.tran.x;
            pose_msg.y = cartesian_pose.tran.y;
            pose_msg.z = cartesian_pose.tran.z;
            pose_msg.rx = cartesian_pose.rpy.rx;
            pose_msg.ry = cartesian_pose.rpy.ry;
            pose_msg.rz = cartesian_pose.rpy.rz;
        }

        publisher_->publish(msg);
    }

    void servo_callback(const jaka_robot_interfaces::msg::ServoJointCommand::SharedPtr msg)
    {
        // 直接保存最新指令，无需加锁
        latest_joint_cmd_left_ = msg->joint_pos_left;
        latest_joint_cmd_right_ = msg->joint_pos_right;
        has_new_cmd_ = true;
    }

    void servo_loop()
    {

        // std::this_thread::sleep_for(std::chrono::seconds(5));    

        sched_param sch;
        sch.sched_priority = 90;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);


        robot_->servo_move_use_joint_NLF(current_servo_vel_, current_servo_acc_, current_servo_jerk_);

        int ret = robot_->servo_move_enable(1, -1);
       
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "servo_move_enable failed, ret = %d", ret);
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "servo_move_enable success.");
        }
        // return;
        timespec next;  
        clock_gettime(CLOCK_REALTIME, &next);



        while (rclcpp::ok() && running_joint_ )
        {
            robot_->edg_recv(&next);
            JointValue jpos_left, jpos_right;
            bool has_data = false;
            // 直接读取最新指令，无需加锁
            if (has_new_cmd_)
            {
                memset(&jpos_left, 0, sizeof(jpos_left));
                memset(&jpos_right, 0, sizeof(jpos_right));
                for (int i = 0; i < AXIS_NUM; ++i)
                {
                    jpos_left.jVal[i] = latest_joint_cmd_left_.joint_values[i];
                    jpos_right.jVal[i] = latest_joint_cmd_right_.joint_values[i];
                }
                has_data = true;
                has_new_cmd_ = false; // 只下发一次，等新指令
            }
            if (has_data)
            {
                int ret_left = robot_->edg_servo_j(0, &jpos_left, MoveMode::ABS);
                int ret_right = robot_->edg_servo_j(1, &jpos_right, MoveMode::ABS);
                RCLCPP_INFO(this->get_logger(), "[1Hz] Sending servo cmd: left: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, right: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, ret_left: %d, ret_right: %d",
                            jpos_left.jVal[0], jpos_left.jVal[1], jpos_left.jVal[2], jpos_left.jVal[3], jpos_left.jVal[4], jpos_left.jVal[5], jpos_left.jVal[6],
                            jpos_right.jVal[0], jpos_right.jVal[1], jpos_right.jVal[2], jpos_right.jVal[3], jpos_right.jVal[4], jpos_right.jVal[5], jpos_right.jVal[6], ret_left, ret_right);
                robot_->edg_send();
            }
            timespec dt;
            dt.tv_nsec = 8000000;
            dt.tv_sec = 0;
            next = timespec_add(next, dt);
            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
        }



    }

    void servo_cart_command_callback(const jaka_robot_interfaces::msg::ServoCartCommand::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_cmd_ = *msg;
    }

    // void servo_loop_pose()
    // {
    //     sched_param sch;
    //     sch.sched_priority = 90;
    //     pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

    //     robot_->servo_move_use_none_filter();

    //     int ret = robot_->servo_move_enable(1, 0);
    //     if (ret != 0)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "servo_move_enable failed, ret = %d", ret);
    //         return;
    //     }
    //     else
    //     {
    //         RCLCPP_INFO(this->get_logger(), "servo_move_enable success.");
    //     }

    //     timespec next;
    //     clock_gettime(CLOCK_REALTIME, &next);

    //     while (rclcpp::ok() && running_pose_)
    //     {
    //         robot_->edg_recv(&next);

    //         CartesianPose pose_left{}, pose_right{};
    //         {
    //             std::lock_guard<std::mutex> lock(pose_mutex_);
    //             pose_left.tran.x = pose_cmd_.end_pose_left.x;
    //             pose_left.tran.y = pose_cmd_.end_pose_left.y;
    //             pose_left.tran.z = pose_cmd_.end_pose_left.z;
    //             pose_left.rpy.rx = pose_cmd_.end_pose_left.rx;
    //             pose_left.rpy.ry = pose_cmd_.end_pose_left.ry;
    //             pose_left.rpy.rz = pose_cmd_.end_pose_left.rz;

    //             pose_right.tran.x = pose_cmd_.end_pose_right.x;
    //             pose_right.tran.y = pose_cmd_.end_pose_right.y;
    //             pose_right.tran.z = pose_cmd_.end_pose_right.z;
    //             pose_right.rpy.rx = pose_cmd_.end_pose_right.rx;
    //             pose_right.rpy.ry = pose_cmd_.end_pose_right.ry;
    //             pose_right.rpy.rz = pose_cmd_.end_pose_right.rz;
    //         }

    //         robot_->edg_servo_p(0, &pose_left, MoveMode::ABS);
    //         robot_->edg_servo_p(1, &pose_right, MoveMode::ABS);
    //         robot_->edg_send();

    //         RCLCPP_INFO(this->get_logger(), "Send pose: left.z=%.2f, right.z=%.2f", pose_left.tran.z, pose_right.tran.z);

    //         timespec dt{0, 8000000}; // 8ms
    //         next = timespec_add(next, dt);
    //         clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, nullptr);
    //     }
    // }
    void servo_loop_pose_left()
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));

        sched_param sch;
        sch.sched_priority = 90;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);

        robot_->servo_move_use_none_filter();

        int ret = robot_->servo_move_enable(1, 0);
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "servo_move_enable failed, ret = %d", ret);
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "servo_move_enable success.");
        }

        timespec next;
        clock_gettime(CLOCK_REALTIME, &next);

        while (rclcpp::ok() && running_pose_)
        {
            robot_->edg_recv(&next);

            CartesianPose pose_left{};
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                pose_left.tran.x = pose_cmd_.end_pose_left.x;
                pose_left.tran.y = pose_cmd_.end_pose_left.y;
                pose_left.tran.z = pose_cmd_.end_pose_left.z;
                pose_left.rpy.rx = pose_cmd_.end_pose_left.rx;
                pose_left.rpy.ry = pose_cmd_.end_pose_left.ry;
                pose_left.rpy.rz = pose_cmd_.end_pose_left.rz;
            }

            // 只给左臂发送伺服命令
            robot_->edg_servo_p(0, &pose_left, MoveMode::ABS);
            robot_->edg_send();

            RCLCPP_INFO(this->get_logger(), "Send left pose: left.z=%.2f", pose_left.tran.z);

            timespec dt{0, 8000000}; // 8ms
            next = timespec_add(next, dt);
            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, nullptr);
        }
    }

    inline timespec timespec_add(timespec t1, timespec t2)
    {
        timespec result;
        result.tv_sec = t1.tv_sec + t2.tv_sec;
        result.tv_nsec = t1.tv_nsec + t2.tv_nsec;
        if (result.tv_nsec >= 1000000000)
        {
            result.tv_sec += 1;
            result.tv_nsec -= 1000000000;
        }
        return result;
    }
    static void robot_error_callback(int error_code)
    {
        std::string msg = get_jaka_error_msg(static_cast<uint32_t>(error_code));
        RCLCPP_ERROR(rclcpp::get_logger("JakaDriverNode"), "robot has error: %d, %s", error_code, msg.c_str());
    }
    void error_monitor_loop()
    {
        while (rclcpp::ok())
        {
            std::this_thread::sleep_for(std::chrono::seconds(2));

            int error[2] = {-1, -1};
            errno_t ret = robot_->robot_is_in_error(error);
            if (ret == ERR_SUCC)
            {
                if (error[0] || error[1])
                {
                    RCLCPP_WARN(this->get_logger(), "robot has error: left = %s, right = %s",
                                error[0] ? "true" : "false", error[1] ? "true" : "false");

                    int left_on_limit, right_on_limit;
                    robot_->robot_is_on_soft_limit(0, &left_on_limit);
                    robot_->robot_is_on_soft_limit(1, &right_on_limit);
                    BOOL in_collision = 0; 
                    robot_->is_in_collision(&in_collision);
                    if (left_on_limit || right_on_limit)
                    {
                        //打印的时候转化成bool
                        RCLCPP_WARN(this->get_logger(), "robot is on soft limit ,Left = %d, Right = %d", bool(left_on_limit), bool(right_on_limit));
                    }
                    if (in_collision)
                    {
                        RCLCPP_WARN(this->get_logger(), "robot is in collision");
                    }
                    
                }
                else
                {
                    RCLCPP_DEBUG(this->get_logger(), "robot has no error");
                }
            }

            ErrorCode code;
            ret = robot_->get_last_error(&code);
            if (ret == ERR_SUCC && code.code != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Last error: code = 0x%x, message = %s", code.code, get_jaka_error_msg(code.code).c_str());
                std::this_thread::sleep_for(std::chrono::seconds(1));
                robot_->clear_error();
                RCLCPP_INFO(this->get_logger(), "clear_error() called.");
            }
        }
    }
    std::shared_ptr<JAKAZuRobot> robot_;
    std::thread error_check_thread_;
    rclcpp::Service<jaka_robot_interfaces::srv::MultiMovJ>::SharedPtr srv_mov_j;
    rclcpp::Service<jaka_robot_interfaces::srv::ServoMovJ>::SharedPtr srv_servo_j;
    rclcpp::Service<jaka_robot_interfaces::srv::MultiMovL>::SharedPtr srv_mov_l;
    rclcpp::Service<jaka_robot_interfaces::srv::JointToCartesian>::SharedPtr srv_joint_to_cartesian_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_error_srv_;
    rclcpp::Publisher<jaka_robot_interfaces::msg::RobotStateDual>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Subscription<ServoJointCommand>::SharedPtr cmd_sub_;
    // rclcpp::Subscription<ServoJointCommand>::SharedPtr servo_joint_sub_;
    rclcpp::Subscription<jaka_robot_interfaces::msg::ServoJointCommand>::SharedPtr servo_sub_;
    rclcpp::Subscription<jaka_robot_interfaces::msg::ServoCartCommand>::SharedPtr cartesian_cmd_sub_;
    // 缓存最新消息
    // jaka_robot_interfaces::msg::ServoJointCommand::SharedPtr latest_cmd_;
    // std::mutex cmd_mutex_;
    // == 伺服相关状态 ==
    // std::deque<ServoJointCommand::_joint_pos_left_type> joint_cmd_left_queue_;
    // std::deque<ServoJointCommand::_joint_pos_right_type> joint_cmd_right_queue_;
    // std::mutex servo_mutex_;   // 关节伺服互斥锁
    ServoJointCommand::_joint_pos_left_type latest_joint_cmd_left_{};
    ServoJointCommand::_joint_pos_right_type latest_joint_cmd_right_{};
    std::atomic<bool> has_new_cmd_{false};
    std::thread servo_thread_; // 关节伺服循环线程

    jaka_robot_interfaces::msg::ServoCartCommand pose_cmd_; // 笛卡尔伺服目标
    std::mutex pose_mutex_;                                 // 笛卡尔伺服互斥锁
    std::thread servo_pose_thread_;                         // 笛卡尔伺服线程（建议独立）

    std::atomic<bool> running_joint_{true};
    std::atomic<bool> running_pose_{true};
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    int current_servo_mode_ = 1; // 1:开, 0:关
    double current_servo_vel_ = 15.0;
    double current_servo_acc_ = 8.0;
    double current_servo_jerk_ = 8.0;

    JointValue jpos_left_;
    JointValue jpos_right_;
};


int main(int argc, char **argv)
{
    //使用multithreadexecutor
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JakaDriverNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
