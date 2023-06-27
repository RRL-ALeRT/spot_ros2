#include <CLI/CLI.hpp>

#include <bosdyn/client/sdk/client_sdk.h>
#include <bosdyn/client/image/image_client.h>
#include <bosdyn/client/util/cli_util.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui.hpp>

#include <tf2_ros/static_transform_broadcaster.h>

cv::Mat imageToMat(const bosdyn::api::Image& image) {
  // Get image dimensions
  int rows = 480;
  int cols = 640;

  // Create a cv::Mat object to hold the image data
  cv::Mat imageMat(rows, cols, CV_8UC1, const_cast<char*>(image.data().data()));

  return imageMat;
}

cv::Mat depthImageToMat(const bosdyn::api::ImageResponse& imageResponse) {
    // Check if the image response is a depth image with PIXEL_FORMAT_DEPTH_U16 pixel format
    if (imageResponse.source().image_type() != bosdyn::api::ImageSource::IMAGE_TYPE_DEPTH) {
        throw std::invalid_argument("Requires an image_type of IMAGE_TYPE_DEPTH.");
    }
    if (imageResponse.shot().image().pixel_format() != bosdyn::api::Image::PIXEL_FORMAT_DEPTH_U16) {
        throw std::invalid_argument("IMAGE_TYPE_DEPTH with an unsupported format, requires PIXEL_FORMAT_DEPTH_U16.");
    }
    if (!imageResponse.source().has_pinhole()) {
        throw std::invalid_argument("Requires a pinhole camera_model.");
    }

    // Get image dimensions and camera intrinsics
    int rows = imageResponse.source().rows();
    int cols = imageResponse.source().cols();
    double fx = imageResponse.source().pinhole().intrinsics().focal_length().x();
    double fy = imageResponse.source().pinhole().intrinsics().focal_length().y();
    double cx = imageResponse.source().pinhole().intrinsics().principal_point().x();
    double cy = imageResponse.source().pinhole().intrinsics().principal_point().y();
    double depthScale = imageResponse.source().depth_scale();

    // Convert the proto representation into a cv::Mat object.
    cv::Mat depthMat(rows, cols, CV_16UC1, const_cast<char*>(imageResponse.shot().image().data().data()));

    // Convert the valid distance data to (x,y,z) values expressed in the sensor frame.
    cv::Mat xyzMat(rows, cols, CV_32FC3);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float z = static_cast<float>(depthMat.at<unsigned short>(i, j)) / depthScale;
            float x = (j - cx) * z / fx;
            float y = (i - cy) * z / fy;
            xyzMat.at<cv::Vec3f>(i, j) = cv::Vec3f(x, y, z);
        }
    }

    return xyzMat;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string cam;

  if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
  {
		RCLCPP_ERROR(rclcpp::get_logger("spot_image"), "ros2 run spot_cpp_ros2 get_images [camera_name] {frontleft, frontright, left, right, back}");
    rclcpp::shutdown();
    return 0;
  }
  else
  {
		cam = argv[argc-1];
  }

  auto node = std::make_shared<rclcpp::Node>(cam + "spot_image");

  // Parse the arguments.
  CLI::App parser{"GetImage"};
  ::bosdyn::client::CommonCLIArgs common_args;
  ::bosdyn::client::AddCommonArguments(parser, common_args);
  CLI11_PARSE(parser, argc, argv);

  // Create a Client SDK object.
  std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk = ::bosdyn::client::CreateStandardSDK(
    "get_image");
  std::cout << "------Created SDK" << std::endl;

  const char* spotIp = std::getenv("SPOT_IP");
  const char* username = std::getenv("BOSDYN_CLIENT_USERNAME");
  const char* password = std::getenv("BOSDYN_CLIENT_PASSWORD");

  if (spotIp == "" || username == "" || password == "")
  {
    std::cerr << "Please set the environment variables {SPOT_IP, BOSDYN_CLIENT_USERNAME, BOSDYN_CLIENT_PASSWORD}" << std::endl;
    return 0;
  }

  // Create a robot instance. A robot instance represents a single user on a single robot.
  ::bosdyn::client::Result<std::unique_ptr<::bosdyn::client::Robot>> robot_result =
    client_sdk->CreateRobot(spotIp);
  if (!robot_result.status)
  {
    std::cerr << "Could not create robot: " << robot_result.status.DebugString() << std::endl;
    return 0;
  }
  std::unique_ptr<::bosdyn::client::Robot> robot = std::move(robot_result.response);
  std::cout << "------Created Robot" << std::endl;

  ::bosdyn::common::Status status = robot->Authenticate(username, password);
  while (!status && rclcpp::ok())
  {
    std::cerr << "Could not authenticate with robot: " << status.DebugString() << " retrying.." << std::endl;
    rclcpp::Rate(0.1).sleep();
    status = robot->Authenticate(username, password);
  }
  std::cout << "------Authenticated with Robot" << std::endl;

  // Create an ImageClient.
  ::bosdyn::client::Result<::bosdyn::client::ImageClient*> image_client_result =
      robot->EnsureServiceClient<::bosdyn::client::ImageClient>(
          ::bosdyn::client::ImageClient::GetDefaultServiceName());
  while (!image_client_result.status && rclcpp::ok())
  {
    std::cerr << "Could not create image client: " << image_client_result.status.DebugString()
              << std::endl;
    rclcpp::Rate(0.1).sleep();
    image_client_result = robot->EnsureServiceClient<::bosdyn::client::ImageClient>(
                          ::bosdyn::client::ImageClient::GetDefaultServiceName());
  }

  if (!rclcpp::ok()) return 0;

  // Call API that accepts a full request
  ::bosdyn::api::GetImageRequest request_message;

  image_transport::ImageTransport it(node);
  auto pub = it.advertise("/Spot/" + cam, 1);

  bosdyn::api::ImageRequest* image_request = request_message.add_image_requests();
  image_request->set_image_source_name(cam + "_fisheye_image");
  image_request->set_quality_percent(65.1);
  image_request->set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);

  bool tf2_published = false;

  auto loop_rate = rclcpp::Rate(10);

  while (rclcpp::ok())
  {
    if (!tf2_published)
    {
      std::shared_future<::bosdyn::client::GetImageResultType> cam_msg =
        image_client_result.response->GetImageAsync(request_message);
          
      // Now get the images from the async calls.
      ::bosdyn::client::GetImageResultType fut_result = cam_msg.get();
      if (!fut_result.status) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Could not get images " << fut_result.status.DebugString());
        continue;
      }

      auto image_responses = fut_result.response.image_responses();
      if (image_responses[0].shot().transforms_snapshot().child_to_parent_edge_map().find(cam) != image_responses[0].shot().transforms_snapshot().child_to_parent_edge_map().end()) {
        auto transform = image_responses[0].shot().transforms_snapshot().child_to_parent_edge_map().at(cam);

        tf2_ros::StaticTransformBroadcaster broadcaster(node);

        // Create a transform message
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = node->now();
        transform_msg.header.frame_id = "body";
        transform_msg.child_frame_id = cam;

        // Set the translation and rotation values
        transform_msg.transform.translation.x = transform.parent_tform_child().position().x();
        transform_msg.transform.translation.y = transform.parent_tform_child().position().y();
        transform_msg.transform.translation.z = transform.parent_tform_child().position().z();

        transform_msg.transform.rotation.x = transform.parent_tform_child().rotation().x();
        transform_msg.transform.rotation.y = transform.parent_tform_child().rotation().y();
        transform_msg.transform.rotation.z = transform.parent_tform_child().rotation().z();
        transform_msg.transform.rotation.w = transform.parent_tform_child().rotation().w();

        // Broadcast the static transform
        broadcaster.sendTransform(transform_msg);

        tf2_published = true;
      }
    }

    if (pub.getNumSubscribers() == 0) continue;

    std::shared_future<::bosdyn::client::GetImageResultType> cam_msg =
      image_client_result.response->GetImageAsync(request_message);
        
    // Now get the images from the async calls.
    ::bosdyn::client::GetImageResultType fut_result = cam_msg.get();
    if (!fut_result.status) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Could not get images " << fut_result.status.DebugString());
      continue;
    }

    auto image_responses = fut_result.response.image_responses();

    std_msgs::msg::Header hdr;
    hdr.stamp = node->get_clock()->now();

    auto img = imageToMat(image_responses[0].shot().image());

    hdr.frame_id = "spot_" + cam;
    sensor_msgs::msg::Image::SharedPtr msg;
    msg = cv_bridge::CvImage(hdr, "mono8", img).toImageMsg();
    
    pub.publish(msg);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}