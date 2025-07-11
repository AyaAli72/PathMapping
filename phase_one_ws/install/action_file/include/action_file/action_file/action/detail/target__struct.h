// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from action_file:action/Target.idl
// generated code does not contain a copyright notice

#ifndef ACTION_FILE__ACTION__DETAIL__TARGET__STRUCT_H_
#define ACTION_FILE__ACTION__DETAIL__TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_point'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in action/Target in the package action_file.
typedef struct action_file__action__Target_Goal
{
  /// Define the target point
  geometry_msgs__msg__Point target_point;
} action_file__action__Target_Goal;

// Struct for a sequence of action_file__action__Target_Goal.
typedef struct action_file__action__Target_Goal__Sequence
{
  action_file__action__Target_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_file__action__Target_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Target in the package action_file.
typedef struct action_file__action__Target_Result
{
  /// Indicate if the target was reached successfully
  bool success;
} action_file__action__Target_Result;

// Struct for a sequence of action_file__action__Target_Result.
typedef struct action_file__action__Target_Result__Sequence
{
  action_file__action__Target_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_file__action__Target_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_position'
// already included above
// #include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in action/Target in the package action_file.
typedef struct action_file__action__Target_Feedback
{
  /// Provide the current position
  geometry_msgs__msg__Point current_position;
} action_file__action__Target_Feedback;

// Struct for a sequence of action_file__action__Target_Feedback.
typedef struct action_file__action__Target_Feedback__Sequence
{
  action_file__action__Target_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_file__action__Target_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "action_file/action/detail/target__struct.h"

/// Struct defined in action/Target in the package action_file.
typedef struct action_file__action__Target_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_file__action__Target_Goal goal;
} action_file__action__Target_SendGoal_Request;

// Struct for a sequence of action_file__action__Target_SendGoal_Request.
typedef struct action_file__action__Target_SendGoal_Request__Sequence
{
  action_file__action__Target_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_file__action__Target_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Target in the package action_file.
typedef struct action_file__action__Target_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} action_file__action__Target_SendGoal_Response;

// Struct for a sequence of action_file__action__Target_SendGoal_Response.
typedef struct action_file__action__Target_SendGoal_Response__Sequence
{
  action_file__action__Target_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_file__action__Target_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Target in the package action_file.
typedef struct action_file__action__Target_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} action_file__action__Target_GetResult_Request;

// Struct for a sequence of action_file__action__Target_GetResult_Request.
typedef struct action_file__action__Target_GetResult_Request__Sequence
{
  action_file__action__Target_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_file__action__Target_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "action_file/action/detail/target__struct.h"

/// Struct defined in action/Target in the package action_file.
typedef struct action_file__action__Target_GetResult_Response
{
  int8_t status;
  action_file__action__Target_Result result;
} action_file__action__Target_GetResult_Response;

// Struct for a sequence of action_file__action__Target_GetResult_Response.
typedef struct action_file__action__Target_GetResult_Response__Sequence
{
  action_file__action__Target_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_file__action__Target_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "action_file/action/detail/target__struct.h"

/// Struct defined in action/Target in the package action_file.
typedef struct action_file__action__Target_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_file__action__Target_Feedback feedback;
} action_file__action__Target_FeedbackMessage;

// Struct for a sequence of action_file__action__Target_FeedbackMessage.
typedef struct action_file__action__Target_FeedbackMessage__Sequence
{
  action_file__action__Target_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_file__action__Target_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACTION_FILE__ACTION__DETAIL__TARGET__STRUCT_H_
