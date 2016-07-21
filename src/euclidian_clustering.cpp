#include <kinect_pcl_obj_detection/euclidian_clustering.hpp>

#include <pcl/segmentation/sac_segmentation.h>

/**
 *   Subscribe to a pointCloud and detects the objects
 *   on the table
 */

int main(int argc, char** argv){
  ros::init(argc, argv, "euclidian_clustering");
  ros::NodeHandle nh, nh_priv("~");
  
  ROS_INFO("Initializing detection...");  
  
  // Get params topics and frames names
  string kinect_topic_name, clusters_topic_name, out_topic_name;
  XmlRpc::XmlRpcValue clipping_rules_bounds;
  bool params_loaded = true;
  params_loaded *= nh_priv.getParam("kinect_topic_name",kinect_topic_name);  
  params_loaded *= nh_priv.getParam("clusters_topic_name",clusters_topic_name);
  //   params_loaded *= nh_priv.getParam("out_topic_name",out_topic_name);
  params_loaded *= nh_priv.getParam("voxel_size",voxel_size_);
  params_loaded *= nh_priv.getParam("min_cluster_size",min_cluster_size_);
  params_loaded *= nh_priv.getParam("minimum_height",minimum_height_);
  params_loaded *= nh_priv.getParam("clipping_rules",clipping_rules_bounds);
  params_loaded *= nh_priv.getParam("clustering_tolerance",clustering_tolerance_);
  params_loaded *= nh_priv.getParam("downsampling",downsampling_);
  
  if(!params_loaded){
    ROS_ERROR("Couldn't find all the required parameters. Closing...");
    ROS_INFO_STREAM("kinect topic: "<<kinect_topic_name);
    ROS_INFO_STREAM("clusters topic: "<<clusters_topic_name);
    ROS_INFO_STREAM("voxel size: "<<voxel_size_);
    ROS_INFO_STREAM("min cluster size: "<<min_cluster_size_);
    ROS_INFO_STREAM("min height: "<<minimum_height_);
    ROS_INFO_STREAM("clipping rules: "<<clipping_rules_bounds);
    ROS_INFO_STREAM("clustering tolerance: "<<clustering_tolerance_);
    ROS_INFO_STREAM("downsampling: "<< downsampling_ ? "true" : "false" );
    return -1;
  }
  
  if (clipping_rules_bounds.size()>0){
    if (clipping_rules_bounds.size()%3)
      ROS_ERROR("Problem in defining the clipping rules.\n Use the following format:\n [x, GT, 1.0, y, LT, 3.1, ...]");
    else{
      clipping_rules_.resize(clipping_rules_bounds.size()/3);
      ClippingRule new_rule;
      for(int i=0; i<clipping_rules_bounds.size()/3;i++){
        new_rule.axis = static_cast<string>(clipping_rules_bounds[i*3]);
        new_rule.op = static_cast<string>(clipping_rules_bounds[i*3+1]);
        new_rule.val = static_cast<double>(clipping_rules_bounds[i*3+2]);
        clipping_rules_.at(i) = new_rule;
      }
    }
    ROS_INFO("%d clipping rules loaded", static_cast<int>(clipping_rules_.size()));
  }
  
  // Initialize PointClouds
  kinects_pc_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  cluster_cloud_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  // Reserve memory for clouds
  kinects_pc_->reserve(10000);
  cluster_cloud_->reserve(10000);
  
  // Ros Subscribers and Publishers
  ros::Subscriber kinect_pc_sub = nh.subscribe<PCMsg>(kinect_topic_name, 1, callback);
  cluster_pc_pub_ = nh.advertise<PCMsg>(clusters_topic_name, 1);
  cubes_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/object_detection/markers", 1);
  
  ROS_INFO("Detection running !"); 
  
  first_frame_ = true;
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& kinect_pc_msg){
  
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*kinect_pc_msg, *kinects_pc_);
  
  if(first_frame_){
    // get transform between the pc frame and the output frame
    get_transform(kinects_pc_->header.frame_id, "base_link", pcl_transform_);
    first_frame_ = false;
  }
  
  // transform the pc to the output frame
  pcl::transformPointCloud(*kinects_pc_, *kinects_pc_, pcl_transform_.translation, pcl_transform_.rotation);
  kinects_pc_->header.frame_id = "base_link";
  
  // Clip pointcloud using the rules defined in params
  pc_clipping(kinects_pc_, clipping_rules_ , kinects_pc_);
  
  // Remove all the NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*kinects_pc_, *kinects_pc_, indices);
  
  // Downsampling the pointCloud
  if(downsampling_)
    pc_downsampling(kinects_pc_, voxel_size_, kinects_pc_);
  
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
//   seg.setOptimizeCoefficients (true);
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setMaxIterations (100);
//   seg.setDistanceThreshold (0.02);
//   
//   int i=0, nr_points = (int) kinects_pc_->points.size ();
//   while (kinects_pc_->points.size () > 0.3 * nr_points)
//   {
//     // Segment the largest planar component from the remaining cloud
//     seg.setInputCloud (kinects_pc_);
//     seg.segment (*inliers, *coefficients);
//     if (inliers->indices.size () == 0)
//     {
//       std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//       break;
//     }
//     
//     // Extract the planar inliers from the input cloud
//     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//     extract.setInputCloud (kinects_pc_);
//     extract.setIndices (inliers);
//     extract.setNegative (false);
//     
//     // Get the points associated with the planar surface
//     extract.filter (*cloud_plane);
// //     std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
//     
//     // Remove the planar inliers, extract the rest
//     extract.setNegative (true);
//     extract.filter (*kinects_pc_);
//   }
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (kinects_pc_);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (kinects_pc_);
  ec.extract (cluster_indices); 
  
  // Gives each cluster a random color
  for(int i=0; i<cluster_indices.size();i++){
    uint8_t r(rand()%255), g(rand()%255), b(rand()%255);
    for(int j=0; j<cluster_indices[i].indices.size();j++){
      kinects_pc_->points[cluster_indices[i].indices[j]].r = r;
      kinects_pc_->points[cluster_indices[i].indices[j]].g = g;
      kinects_pc_->points[cluster_indices[i].indices[j]].b = b;
    }
  }
  
  // Publishing clusters
  cluster_pc_pub_.publish(*kinects_pc_);

}

void get_transform(string in_frame, string out_frame, pclTransform &pc_transform){
  
  tf::TransformListener trans_listener;
  tf::StampedTransform stampedTransform;
  
  try{
    trans_listener.waitForTransform(out_frame, in_frame, ros::Time(0), ros::Duration(10.0));
    trans_listener.lookupTransform(out_frame, in_frame, ros::Time(0), stampedTransform);
  }
  catch(tf::TransformException &ex){
    cout << ex.what() << endl;
    ROS_ERROR("%s", ex.what());
  }
  tf::vectorTFToEigen(stampedTransform.getOrigin(), pc_transform.translation);      
  tf::quaternionTFToEigen(stampedTransform.getRotation(), pc_transform.rotation);
  
  return;
}
