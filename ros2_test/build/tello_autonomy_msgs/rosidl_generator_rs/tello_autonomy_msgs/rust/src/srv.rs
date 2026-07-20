#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to tello_autonomy_msgs__srv__NbvPlan_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NbvPlan_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

}



impl Default for NbvPlan_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::NbvPlan_Request::default())
  }
}

impl rosidl_runtime_rs::Message for NbvPlan_Request {
  type RmwMsg = super::srv::rmw::NbvPlan_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
    }
  }
}


// Corresponds to tello_autonomy_msgs__srv__NbvPlan_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NbvPlan_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub path: Vec<geometry_msgs::msg::Pose>,

}



impl Default for NbvPlan_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::NbvPlan_Response::default())
  }
}

impl rosidl_runtime_rs::Message for NbvPlan_Response {
  type RmwMsg = super::srv::rmw::NbvPlan_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path
          .into_iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path
          .iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: msg.path
          .into_iter()
          .map(geometry_msgs::msg::Pose::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to tello_autonomy_msgs__srv__SearchPlan_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchPlan_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub start: geometry_msgs::msg::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: geometry_msgs::msg::PoseStamped,

}



impl Default for SearchPlan_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SearchPlan_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SearchPlan_Request {
  type RmwMsg = super::srv::rmw::SearchPlan_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.start)).into_owned(),
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(msg.goal)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        start: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start)).into_owned(),
        goal: geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(&msg.goal)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      start: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.start),
      goal: geometry_msgs::msg::PoseStamped::from_rmw_message(msg.goal),
    }
  }
}


// Corresponds to tello_autonomy_msgs__srv__SearchPlan_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SearchPlan_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub path: Vec<geometry_msgs::msg::PoseStamped>,

}



impl Default for SearchPlan_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SearchPlan_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SearchPlan_Response {
  type RmwMsg = super::srv::rmw::SearchPlan_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        path: msg.path
          .into_iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        path: msg.path
          .iter()
          .map(|elem| geometry_msgs::msg::PoseStamped::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      path: msg.path
          .into_iter()
          .map(geometry_msgs::msg::PoseStamped::from_rmw_message)
          .collect(),
    }
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


