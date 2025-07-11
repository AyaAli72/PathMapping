// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from action_file:action/Target.idl
// generated code does not contain a copyright notice

#ifndef ACTION_FILE__ACTION__DETAIL__TARGET__BUILDER_HPP_
#define ACTION_FILE__ACTION__DETAIL__TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "action_file/action/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace action_file
{

namespace action
{

namespace builder
{

class Init_Target_Goal_target_point
{
public:
  Init_Target_Goal_target_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_file::action::Target_Goal target_point(::action_file::action::Target_Goal::_target_point_type arg)
  {
    msg_.target_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_file::action::Target_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_file::action::Target_Goal>()
{
  return action_file::action::builder::Init_Target_Goal_target_point();
}

}  // namespace action_file


namespace action_file
{

namespace action
{

namespace builder
{

class Init_Target_Result_success
{
public:
  Init_Target_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_file::action::Target_Result success(::action_file::action::Target_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_file::action::Target_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_file::action::Target_Result>()
{
  return action_file::action::builder::Init_Target_Result_success();
}

}  // namespace action_file


namespace action_file
{

namespace action
{

namespace builder
{

class Init_Target_Feedback_current_position
{
public:
  Init_Target_Feedback_current_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_file::action::Target_Feedback current_position(::action_file::action::Target_Feedback::_current_position_type arg)
  {
    msg_.current_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_file::action::Target_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_file::action::Target_Feedback>()
{
  return action_file::action::builder::Init_Target_Feedback_current_position();
}

}  // namespace action_file


namespace action_file
{

namespace action
{

namespace builder
{

class Init_Target_SendGoal_Request_goal
{
public:
  explicit Init_Target_SendGoal_Request_goal(::action_file::action::Target_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::action_file::action::Target_SendGoal_Request goal(::action_file::action::Target_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_file::action::Target_SendGoal_Request msg_;
};

class Init_Target_SendGoal_Request_goal_id
{
public:
  Init_Target_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_SendGoal_Request_goal goal_id(::action_file::action::Target_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Target_SendGoal_Request_goal(msg_);
  }

private:
  ::action_file::action::Target_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_file::action::Target_SendGoal_Request>()
{
  return action_file::action::builder::Init_Target_SendGoal_Request_goal_id();
}

}  // namespace action_file


namespace action_file
{

namespace action
{

namespace builder
{

class Init_Target_SendGoal_Response_stamp
{
public:
  explicit Init_Target_SendGoal_Response_stamp(::action_file::action::Target_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::action_file::action::Target_SendGoal_Response stamp(::action_file::action::Target_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_file::action::Target_SendGoal_Response msg_;
};

class Init_Target_SendGoal_Response_accepted
{
public:
  Init_Target_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_SendGoal_Response_stamp accepted(::action_file::action::Target_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Target_SendGoal_Response_stamp(msg_);
  }

private:
  ::action_file::action::Target_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_file::action::Target_SendGoal_Response>()
{
  return action_file::action::builder::Init_Target_SendGoal_Response_accepted();
}

}  // namespace action_file


namespace action_file
{

namespace action
{

namespace builder
{

class Init_Target_GetResult_Request_goal_id
{
public:
  Init_Target_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_file::action::Target_GetResult_Request goal_id(::action_file::action::Target_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_file::action::Target_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_file::action::Target_GetResult_Request>()
{
  return action_file::action::builder::Init_Target_GetResult_Request_goal_id();
}

}  // namespace action_file


namespace action_file
{

namespace action
{

namespace builder
{

class Init_Target_GetResult_Response_result
{
public:
  explicit Init_Target_GetResult_Response_result(::action_file::action::Target_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::action_file::action::Target_GetResult_Response result(::action_file::action::Target_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_file::action::Target_GetResult_Response msg_;
};

class Init_Target_GetResult_Response_status
{
public:
  Init_Target_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_GetResult_Response_result status(::action_file::action::Target_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Target_GetResult_Response_result(msg_);
  }

private:
  ::action_file::action::Target_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_file::action::Target_GetResult_Response>()
{
  return action_file::action::builder::Init_Target_GetResult_Response_status();
}

}  // namespace action_file


namespace action_file
{

namespace action
{

namespace builder
{

class Init_Target_FeedbackMessage_feedback
{
public:
  explicit Init_Target_FeedbackMessage_feedback(::action_file::action::Target_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::action_file::action::Target_FeedbackMessage feedback(::action_file::action::Target_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_file::action::Target_FeedbackMessage msg_;
};

class Init_Target_FeedbackMessage_goal_id
{
public:
  Init_Target_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_FeedbackMessage_feedback goal_id(::action_file::action::Target_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Target_FeedbackMessage_feedback(msg_);
  }

private:
  ::action_file::action::Target_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_file::action::Target_FeedbackMessage>()
{
  return action_file::action::builder::Init_Target_FeedbackMessage_goal_id();
}

}  // namespace action_file

#endif  // ACTION_FILE__ACTION__DETAIL__TARGET__BUILDER_HPP_
