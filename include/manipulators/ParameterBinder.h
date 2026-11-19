#ifndef PARAMETER_BINDER_H
#define PARAMETER_BINDER_H

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

/*!
    @brief Binds ROS 2 parameters to existing variables.

    Each bound parameter is declared on the node with a ParameterDescriptor, so rclcpp itself
    enforces read_only, type and range constraints on every set request. After a set request
    succeeds, the bound variable is updated (post-set callback) and the optional on_change
    callback is invoked.

    Usage:
        binder.bind("max_speed_ee", 1.0, &max_speed_ee_, ParameterBinder::descriptor("...").range(0.0, 10.0));
*/
class ParameterBinder
{
    // Prevents template deduction through the on_change argument (C++17 replacement for std::type_identity)
    template <typename T> struct NonDeduced { using type = T; };

public:
    // Small builder around rcl_interfaces::msg::ParameterDescriptor
    struct Descriptor
    {
        rcl_interfaces::msg::ParameterDescriptor msg;

        Descriptor &readOnly(bool ro = true) { msg.read_only = ro; return *this; }

        Descriptor &range(double from, double to, double step = 0.0)
        {
            rcl_interfaces::msg::FloatingPointRange r;
            r.from_value = from;
            r.to_value = to;
            r.step = step;
            msg.floating_point_range = {r};
            return *this;
        }

        Descriptor &intRange(int64_t from, int64_t to, uint64_t step = 1)
        {
            rcl_interfaces::msg::IntegerRange r;
            r.from_value = from;
            r.to_value = to;
            r.step = step;
            msg.integer_range = {r};
            return *this;
        }

        operator const rcl_interfaces::msg::ParameterDescriptor &() const { return msg; }
    };

    static Descriptor descriptor(const std::string &description = "")
    {
        Descriptor d;
        d.msg.description = description;
        return d;
    }

    explicit ParameterBinder(rclcpp::Node *node) : node_(node)
    {
        post_set_handle_ = node_->add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) {
                for (const auto &param : parameters)
                {
                    auto it = bindings_.find(param.get_name());
                    if (it == bindings_.end()) continue;
                    for (auto &apply : it->second) apply(param.get_parameter_value());
                }
            });
    }

    // Non copyable/movable: the callback captures `this`
    ParameterBinder(const ParameterBinder &) = delete;
    ParameterBinder &operator=(const ParameterBinder &) = delete;

    /*!
        @brief Declares (if needed) the parameter `name` and keeps `*var` in sync with it.
        @param name Parameter name
        @param default_value Value used when no override is given (YAML/launch)
        @param var Pointer to the variable to keep updated, must outlive this binder
        @param desc Parameter descriptor (description, read_only, ranges)
        @param on_change Optional callback invoked with the new value at bind time and after every update
        @throws rclcpp::exceptions::InvalidParameterTypeException if the override has the wrong type,
                rclcpp::exceptions::InvalidParameterValueException if it violates the descriptor
    */
    template <typename T>
    void bind(const std::string &name,
              const T &default_value,
              T *var,
              const rcl_interfaces::msg::ParameterDescriptor &desc = rcl_interfaces::msg::ParameterDescriptor(),
              std::function<void(const typename NonDeduced<T>::type &)> on_change = nullptr)
    {
        if (!node_->has_parameter(name))
        {
            try
            {
                node_->declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL(node_->get_logger(), "Invalid value for parameter '%s': %s", name.c_str(), e.what());
                throw;
            }
        }

        auto apply = [var, on_change](const rclcpp::ParameterValue &value) {
            *var = static_cast<T>(value.get<T>());
            if (on_change) on_change(*var);
        };

        apply(node_->get_parameter(name).get_parameter_value());
        bindings_[name].push_back(apply);
    }

    // Shortcut for parameters that can only be set at startup
    template <typename T>
    void bindReadOnly(const std::string &name, const T &default_value, T *var, const std::string &description = "")
    {
        bind(name, default_value, var, descriptor(description).readOnly());
    }

private:
    rclcpp::Node *node_;
    std::unordered_map<std::string, std::vector<std::function<void(const rclcpp::ParameterValue &)>>> bindings_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;
};

#endif // PARAMETER_BINDER_H
