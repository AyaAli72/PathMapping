// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from action_file:action/Target.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "action_file/action/detail/target__struct.h"
#include "action_file/action/detail/target__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_Goal_type_support_ids_t;

static const _Target_Goal_type_support_ids_t _Target_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_Goal_type_support_symbol_names_t _Target_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_Goal)),
  }
};

typedef struct _Target_Goal_type_support_data_t
{
  void * data[2];
} _Target_Goal_type_support_data_t;

static _Target_Goal_type_support_data_t _Target_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_Goal_message_typesupport_map = {
  2,
  "action_file",
  &_Target_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Target_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Target_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Target_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_Goal)() {
  return &::action_file::action::rosidl_typesupport_c::Target_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "action_file/action/detail/target__struct.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_Result_type_support_ids_t;

static const _Target_Result_type_support_ids_t _Target_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_Result_type_support_symbol_names_t _Target_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_Result)),
  }
};

typedef struct _Target_Result_type_support_data_t
{
  void * data[2];
} _Target_Result_type_support_data_t;

static _Target_Result_type_support_data_t _Target_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_Result_message_typesupport_map = {
  2,
  "action_file",
  &_Target_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Target_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Target_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Target_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_Result)() {
  return &::action_file::action::rosidl_typesupport_c::Target_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "action_file/action/detail/target__struct.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_Feedback_type_support_ids_t;

static const _Target_Feedback_type_support_ids_t _Target_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_Feedback_type_support_symbol_names_t _Target_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_Feedback)),
  }
};

typedef struct _Target_Feedback_type_support_data_t
{
  void * data[2];
} _Target_Feedback_type_support_data_t;

static _Target_Feedback_type_support_data_t _Target_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_Feedback_message_typesupport_map = {
  2,
  "action_file",
  &_Target_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Target_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Target_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Target_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_Feedback)() {
  return &::action_file::action::rosidl_typesupport_c::Target_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "action_file/action/detail/target__struct.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_SendGoal_Request_type_support_ids_t;

static const _Target_SendGoal_Request_type_support_ids_t _Target_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_SendGoal_Request_type_support_symbol_names_t _Target_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_SendGoal_Request)),
  }
};

typedef struct _Target_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Target_SendGoal_Request_type_support_data_t;

static _Target_SendGoal_Request_type_support_data_t _Target_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_SendGoal_Request_message_typesupport_map = {
  2,
  "action_file",
  &_Target_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Target_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Target_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Target_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_SendGoal_Request)() {
  return &::action_file::action::rosidl_typesupport_c::Target_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "action_file/action/detail/target__struct.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_SendGoal_Response_type_support_ids_t;

static const _Target_SendGoal_Response_type_support_ids_t _Target_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_SendGoal_Response_type_support_symbol_names_t _Target_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_SendGoal_Response)),
  }
};

typedef struct _Target_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Target_SendGoal_Response_type_support_data_t;

static _Target_SendGoal_Response_type_support_data_t _Target_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_SendGoal_Response_message_typesupport_map = {
  2,
  "action_file",
  &_Target_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Target_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Target_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Target_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_SendGoal_Response)() {
  return &::action_file::action::rosidl_typesupport_c::Target_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_SendGoal_type_support_ids_t;

static const _Target_SendGoal_type_support_ids_t _Target_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_SendGoal_type_support_symbol_names_t _Target_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_SendGoal)),
  }
};

typedef struct _Target_SendGoal_type_support_data_t
{
  void * data[2];
} _Target_SendGoal_type_support_data_t;

static _Target_SendGoal_type_support_data_t _Target_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_SendGoal_service_typesupport_map = {
  2,
  "action_file",
  &_Target_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Target_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Target_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Target_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_SendGoal)() {
  return &::action_file::action::rosidl_typesupport_c::Target_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "action_file/action/detail/target__struct.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_GetResult_Request_type_support_ids_t;

static const _Target_GetResult_Request_type_support_ids_t _Target_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_GetResult_Request_type_support_symbol_names_t _Target_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_GetResult_Request)),
  }
};

typedef struct _Target_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Target_GetResult_Request_type_support_data_t;

static _Target_GetResult_Request_type_support_data_t _Target_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_GetResult_Request_message_typesupport_map = {
  2,
  "action_file",
  &_Target_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Target_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Target_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Target_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_GetResult_Request)() {
  return &::action_file::action::rosidl_typesupport_c::Target_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "action_file/action/detail/target__struct.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_GetResult_Response_type_support_ids_t;

static const _Target_GetResult_Response_type_support_ids_t _Target_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_GetResult_Response_type_support_symbol_names_t _Target_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_GetResult_Response)),
  }
};

typedef struct _Target_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Target_GetResult_Response_type_support_data_t;

static _Target_GetResult_Response_type_support_data_t _Target_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_GetResult_Response_message_typesupport_map = {
  2,
  "action_file",
  &_Target_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Target_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Target_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Target_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_GetResult_Response)() {
  return &::action_file::action::rosidl_typesupport_c::Target_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_GetResult_type_support_ids_t;

static const _Target_GetResult_type_support_ids_t _Target_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_GetResult_type_support_symbol_names_t _Target_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_GetResult)),
  }
};

typedef struct _Target_GetResult_type_support_data_t
{
  void * data[2];
} _Target_GetResult_type_support_data_t;

static _Target_GetResult_type_support_data_t _Target_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_GetResult_service_typesupport_map = {
  2,
  "action_file",
  &_Target_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Target_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Target_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Target_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_GetResult)() {
  return &::action_file::action::rosidl_typesupport_c::Target_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "action_file/action/detail/target__struct.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace action_file
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Target_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Target_FeedbackMessage_type_support_ids_t;

static const _Target_FeedbackMessage_type_support_ids_t _Target_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Target_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Target_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Target_FeedbackMessage_type_support_symbol_names_t _Target_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_file, action, Target_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_file, action, Target_FeedbackMessage)),
  }
};

typedef struct _Target_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Target_FeedbackMessage_type_support_data_t;

static _Target_FeedbackMessage_type_support_data_t _Target_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Target_FeedbackMessage_message_typesupport_map = {
  2,
  "action_file",
  &_Target_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Target_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Target_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Target_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Target_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace action_file

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, action_file, action, Target_FeedbackMessage)() {
  return &::action_file::action::rosidl_typesupport_c::Target_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "action_file/action/target.h"
// already included above
// #include "action_file/action/detail/target__type_support.h"

static rosidl_action_type_support_t _action_file__action__Target__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, action_file, action, Target)()
{
  // Thread-safe by always writing the same values to the static struct
  _action_file__action__Target__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_file, action, Target_SendGoal)();
  _action_file__action__Target__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_file, action, Target_GetResult)();
  _action_file__action__Target__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _action_file__action__Target__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_file, action, Target_FeedbackMessage)();
  _action_file__action__Target__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_action_file__action__Target__typesupport_c;
}

#ifdef __cplusplus
}
#endif
