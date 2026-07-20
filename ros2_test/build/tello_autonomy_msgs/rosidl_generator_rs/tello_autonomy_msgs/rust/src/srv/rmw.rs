#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "tello_autonomy_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tello_autonomy_msgs__srv__NbvPlan_Request() -> *const std::ffi::c_void;
}

#[link(name = "tello_autonomy_msgs__rosidl_generator_c")]
extern "C" {
    fn tello_autonomy_msgs__srv__NbvPlan_Request__init(msg: *mut NbvPlan_Request) -> bool;
    fn tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NbvPlan_Request>, size: usize) -> bool;
    fn tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NbvPlan_Request>);
    fn tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NbvPlan_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<NbvPlan_Request>) -> bool;
}

// Corresponds to tello_autonomy_msgs__srv__NbvPlan_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NbvPlan_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

}



impl Default for NbvPlan_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tello_autonomy_msgs__srv__NbvPlan_Request__init(&mut msg as *mut _) {
        panic!("Call to tello_autonomy_msgs__srv__NbvPlan_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NbvPlan_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__NbvPlan_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NbvPlan_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NbvPlan_Request where Self: Sized {
  const TYPE_NAME: &'static str = "tello_autonomy_msgs/srv/NbvPlan_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tello_autonomy_msgs__srv__NbvPlan_Request() }
  }
}


#[link(name = "tello_autonomy_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tello_autonomy_msgs__srv__NbvPlan_Response() -> *const std::ffi::c_void;
}

#[link(name = "tello_autonomy_msgs__rosidl_generator_c")]
extern "C" {
    fn tello_autonomy_msgs__srv__NbvPlan_Response__init(msg: *mut NbvPlan_Response) -> bool;
    fn tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NbvPlan_Response>, size: usize) -> bool;
    fn tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NbvPlan_Response>);
    fn tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NbvPlan_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<NbvPlan_Response>) -> bool;
}

// Corresponds to tello_autonomy_msgs__srv__NbvPlan_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NbvPlan_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Pose>,

}



impl Default for NbvPlan_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tello_autonomy_msgs__srv__NbvPlan_Response__init(&mut msg as *mut _) {
        panic!("Call to tello_autonomy_msgs__srv__NbvPlan_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NbvPlan_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__NbvPlan_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NbvPlan_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NbvPlan_Response where Self: Sized {
  const TYPE_NAME: &'static str = "tello_autonomy_msgs/srv/NbvPlan_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tello_autonomy_msgs__srv__NbvPlan_Response() }
  }
}


#[link(name = "tello_autonomy_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tello_autonomy_msgs__srv__SearchPlan_Request() -> *const std::ffi::c_void;
}

#[link(name = "tello_autonomy_msgs__rosidl_generator_c")]
extern "C" {
    fn tello_autonomy_msgs__srv__SearchPlan_Request__init(msg: *mut SearchPlan_Request) -> bool;
    fn tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchPlan_Request>, size: usize) -> bool;
    fn tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchPlan_Request>);
    fn tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchPlan_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchPlan_Request>) -> bool;
}

// Corresponds to tello_autonomy_msgs__srv__SearchPlan_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchPlan_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::rmw::PoseStamped,

}



impl Default for SearchPlan_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tello_autonomy_msgs__srv__SearchPlan_Request__init(&mut msg as *mut _) {
        panic!("Call to tello_autonomy_msgs__srv__SearchPlan_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchPlan_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__SearchPlan_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchPlan_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchPlan_Request where Self: Sized {
  const TYPE_NAME: &'static str = "tello_autonomy_msgs/srv/SearchPlan_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tello_autonomy_msgs__srv__SearchPlan_Request() }
  }
}


#[link(name = "tello_autonomy_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__tello_autonomy_msgs__srv__SearchPlan_Response() -> *const std::ffi::c_void;
}

#[link(name = "tello_autonomy_msgs__rosidl_generator_c")]
extern "C" {
    fn tello_autonomy_msgs__srv__SearchPlan_Response__init(msg: *mut SearchPlan_Response) -> bool;
    fn tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SearchPlan_Response>, size: usize) -> bool;
    fn tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SearchPlan_Response>);
    fn tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SearchPlan_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SearchPlan_Response>) -> bool;
}

// Corresponds to tello_autonomy_msgs__srv__SearchPlan_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchPlan_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub path: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::PoseStamped>,

}



impl Default for SearchPlan_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !tello_autonomy_msgs__srv__SearchPlan_Response__init(&mut msg as *mut _) {
        panic!("Call to tello_autonomy_msgs__srv__SearchPlan_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SearchPlan_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { tello_autonomy_msgs__srv__SearchPlan_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SearchPlan_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SearchPlan_Response where Self: Sized {
  const TYPE_NAME: &'static str = "tello_autonomy_msgs/srv/SearchPlan_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__tello_autonomy_msgs__srv__SearchPlan_Response() }
  }
}






#[link(name = "tello_autonomy_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__tello_autonomy_msgs__srv__NbvPlan() -> *const std::ffi::c_void;
}

// Corresponds to tello_autonomy_msgs__srv__NbvPlan
#[allow(missing_docs, non_camel_case_types)]
pub struct NbvPlan;

impl rosidl_runtime_rs::Service for NbvPlan {
    type Request = NbvPlan_Request;
    type Response = NbvPlan_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__tello_autonomy_msgs__srv__NbvPlan() }
    }
}




#[link(name = "tello_autonomy_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__tello_autonomy_msgs__srv__SearchPlan() -> *const std::ffi::c_void;
}

// Corresponds to tello_autonomy_msgs__srv__SearchPlan
#[allow(missing_docs, non_camel_case_types)]
pub struct SearchPlan;

impl rosidl_runtime_rs::Service for SearchPlan {
    type Request = SearchPlan_Request;
    type Response = SearchPlan_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__tello_autonomy_msgs__srv__SearchPlan() }
    }
}


