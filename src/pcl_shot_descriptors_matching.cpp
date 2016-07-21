#include <kinect_pcl_obj_detection/pcl_shot_descriptors_matching.hpp>

float rf_rad = 0.016;


int main(int argc, char** argv){
  ros::init(argc, argv, "pcl_viewer");
  ros::NodeHandle nh, nh_priv("~");
  
  ROS_INFO("Initializing viewer...");  
  
  // Initialize PointClouds
  kinects_pc_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  cluster_cloud_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  normals_ = boost::shared_ptr<pcl::PointCloud<NormalType> >(new pcl::PointCloud<NormalType>);
  robot_normals_ = boost::shared_ptr<pcl::PointCloud<NormalType> >(new pcl::PointCloud<NormalType>);
  robot_pc_ = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  nh_priv.getParam("rf_rad",rf_rad);
  // Reserve memory for clouds
  kinects_pc_->reserve(10000);
  cluster_cloud_->reserve(10000);
  normals_->reserve(10000);
  robot_normals_->reserve(10000);
  
  // Ros Subscribers and Publishers
  ros::Subscriber kinect_pc_sub = nh.subscribe<PCMsg>("/kinectV2/qhd/points", 1, callback);
  cluster_pc_pub_ = nh.advertise<PCMsg>("/kinectV2/qhd/test_points",1);
//   ros::Subscriber kinect_pc_sub = nh.subscribe<PCMsg>("/kinect1/depth_registered/points", 1, callback);
//   cluster_pc_pub_ = nh.advertise<PCMsg>("/kinect1/depth_registered/points_downsampled",1);
//   ros::Subscriber kinect_pc_sub = nh.subscribe<PCMsg>("/robot_model_to_pointcloud/robot_cloud2", 1, callback);
//   cluster_pc_pub_ = nh.advertise<PCMsg>("/robot_model_to_pointcloud/robot_cloud2_downsampled",1);
  
  // Global params init
  first_frame_ = true;
  
  // Load robot model
  robot_pcd_path_ = ros::package::getPath("kinect_pcl_obj_detection") + "/data/robot_0.005.pcd";
  pcl::io::loadPCDFile(robot_pcd_path_,*robot_pc_);
  
  // Shitf robot model
  pclTransform pcl_shift;
  tf::StampedTransform tf_shift;
  tf::Vector3 vec3_shitf(0.0, 0.5, 0.0);
  tf_shift.setOrigin(vec3_shitf);
  tf::vectorTFToEigen(tf_shift.getOrigin(), pcl_shift.translation);      
  tf::quaternionTFToEigen(tf_shift.getRotation(), pcl_shift.rotation);
  pcl::transformPointCloud(*robot_pc_, *robot_pc_, pcl_shift.translation, pcl_shift.rotation);
  
  // Robot normals 
  norm_est_.setKSearch(10);
  norm_est_.setInputCloud (robot_pc_);
  norm_est_.compute (*robot_normals_);
  
  viewer_robot_.addPointCloud(robot_pc_, "robot");
  viewer_robot_.addPointCloudNormals<PointType, NormalType>(robot_pc_, robot_normals_, 1, 0.05, "robot_normals");
  
  // PCL viewer
  viewer_.setBackgroundColor (0.0, 0.0, 0.0);
  viewer_.addPointCloud(kinects_pc_,"sample_cloud");
  viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_cloud");
  viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "sample_cloud");  
  viewer_.initCameraParameters ();
  
  ros::spin();
  return 0;
}

void callback(const PCMsg::ConstPtr& kinect_pc_msg){
  
  // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg(*kinect_pc_msg, *kinects_pc_);

  // Remove NaNs
  vector<int> indices;
  pcl::removeNaNFromPointCloud<PointType>(*kinects_pc_, *kinects_pc_, indices);
  
  // Downsampling
  pc_downsampling<PointType>(kinects_pc_, 0.005,cluster_cloud_);
  
  // Get transform between pc frame and base_link
  if(first_frame_){
    // get transform between the pc frame and the output frame
    get_transform(kinects_pc_->header.frame_id, "base_link", pc_transform_);
    first_frame_ = false;
  }
  
  // Transform the pc to base_link
  pcl::transformPointCloud(*cluster_cloud_, *cluster_cloud_, pc_transform_.translation, pc_transform_.rotation);
  kinects_pc_->header.frame_id = "base_link";
  
  // Clip the pointcloud
  pcl::ConditionAnd<PointType>::Ptr height_cond (new pcl::ConditionAnd<PointType> ());
  pcl::ConditionalRemoval<PointType> condrem (height_cond);
  height_cond->addComparison ( pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::GT, 0.15)));
  height_cond->addComparison ( pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::GT, -1.0)));
  height_cond->addComparison ( pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::LT, 1.0)));
  height_cond->addComparison ( pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::LT, 1.5)));
  height_cond->addComparison ( pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::GT, -0.5)));
  condrem.setInputCloud (cluster_cloud_);
  condrem.setKeepOrganized(true);
  condrem.filter (*cluster_cloud_);

  // Estimate normals
  norm_est_.setKSearch(20);
  norm_est_.setInputCloud (cluster_cloud_);
  norm_est_.compute (*normals_);

  // PCL Viewer
  if(first_frame_)
    viewer_.addPointCloudNormals<PointType, NormalType>(cluster_cloud_, normals_, 1, 0.05, "normals");
  else{
    viewer_.removePointCloud("normals");
    viewer_.addPointCloudNormals<PointType, NormalType>(cluster_cloud_, normals_, 1, 0.05, "normals");
  }
  viewer_.updatePointCloud(cluster_cloud_,"sample_cloud");
  viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
  viewer_.spinOnce();  
  
 
  bool use_hough_ (true), show_correspondences_(true), show_keypoints_(true);
  float model_ss_ (0.005f);
  float scene_ss_ (0.005f);
  float descr_rad_ (0.036f);
  //float rf_rad_ (0.1f);
  float rf_rad_(rf_rad);
  float cg_size_ (0.2f);
  float cg_thresh_ (3.0f);
  int knn = 1;
  
  
  pcl::PointCloud<int> model_keypoints_indices, scene_keypoints_indices;
  pcl::PointCloud<PointType>::Ptr model_keypoints, scene_keypoints;
  model_keypoints = boost::shared_ptr<pcl::PointCloud<PointType> >(new pcl::PointCloud<PointType> );
  scene_keypoints = boost::shared_ptr<pcl::PointCloud<PointType> >(new pcl::PointCloud<PointType> );
  
  
  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (robot_pc_);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.compute(model_keypoints_indices);
  std::cout << "Model total points: " << robot_pc_->size () << "; Selected Keypoints: " << model_keypoints_indices.size () << std::endl;
  
  uniform_sampling.setInputCloud (cluster_cloud_);
  uniform_sampling.setRadiusSearch (scene_ss_);;
  uniform_sampling.compute(scene_keypoints_indices);
  std::cout << "Scene total points: " << cluster_cloud_->size () << "; Selected Keypoints: " << scene_keypoints_indices.size () << std::endl;
  
  // Use Keypoint indices to make a PointCloud
  model_keypoints->points.resize(model_keypoints_indices.points.size ());
  for (size_t i=0; i<model_keypoints_indices.points.size (); ++i)
    model_keypoints->points[i].getVector3fMap () = robot_pc_->points[model_keypoints_indices.points[i]].getVector3fMap();
  
  scene_keypoints->points.resize (scene_keypoints_indices.points.size ());
  for (size_t i=0; i<scene_keypoints_indices.points.size (); ++i)
    scene_keypoints->points[i].getVector3fMap () = cluster_cloud_->points[scene_keypoints_indices.points[i]].getVector3fMap();
    
  std::cout<<"keypoints converted"<<std::endl;
  
  //
  //  Compute Descriptor for keypoints
  //
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
  
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  //descr_est.setKSearch(10);
  descr_est.setRadiusSearch (descr_rad_);
  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (robot_normals_);
  descr_est.setSearchSurface (robot_pc_);
  descr_est.compute (*model_descriptors);
  std::cout<<"SHOTEstimationOMP done for model"<<std::endl;  
  
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est2;
//   descr_est2.setKSearch(10);
  descr_est2.setRadiusSearch (descr_rad_);
  descr_est2.setInputCloud (scene_keypoints);
  descr_est2.setInputNormals (normals_);
  descr_est2.setSearchSurface (cluster_cloud_);
  descr_est2.compute (*scene_descriptors);
  std::cout<<"SHOTEstimationOMP done for scene"<<std::endl;  
  
  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());  
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);
  
  std::cout<<"KdTreeFLANN searched"<<std::endl;
  
  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
//   for (size_t i = 0; i < scene_descriptors->size (); ++i)
//   {
//     std::vector<int> neigh_indices (1);
//     std::vector<float> neigh_sqr_dists (1);
//     if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
//     {
//       continue;
//     }
//     int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
// //     if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//     if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//     {
//       pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//       model_scene_corrs->push_back (corr);
//     }
//   }

  for (size_t i = 0; i < scene_descriptors->size (); ++i) 
  { 
    std::vector<int> neigh_indices (knn); 
    std::vector<float> neigh_sqr_dists (knn); 
    if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs 
    { 
      continue; 
    } 
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), knn, neigh_indices, neigh_sqr_dists); 
    for(int k = 0; k < found_neighs; k++) 
    { 
      if(found_neighs == 1 && neigh_sqr_dists[k] < 0.1f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design) 
      { 
        pcl::Correspondence corr (neigh_indices[k], static_cast<int> (i), neigh_sqr_dists[k]); 
        model_scene_corrs->push_back (corr); 
      } 
    } 
  } 
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
  
  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;
  
  std::cout<<"Actual clustering done"<<std::endl;
  
  //  Using Hough3D
  if (use_hough_)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());
    
    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);
    
    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (robot_normals_);
    rf_est.setSearchSurface (robot_pc_);
    rf_est.compute (*model_rf);
    
    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (normals_);
    rf_est.setSearchSurface (cluster_cloud_);
    rf_est.compute (*scene_rf);
    
    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);
    
    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);
    
    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else // Using GeometricConsistency
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);
    
    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);
    
    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }
  
  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
    
    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
    
    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }
  
  //  Visualization
  // White cloud: scene
  // Yellow cloud: off_scene_MODEL
  // Blue cloud: scene keypoints and off_scene_MODEL keypoints
  // Red cloud: rotated model
  // Green lines: correspondences between

  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (cluster_cloud_, "scene_cloud");

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  if (show_correspondences_ || show_keypoints_)
  {
//       We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*robot_pc_, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*robot_pc_, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    if (show_correspondences_)
    {
      for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

//           We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  exit(0);
  
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
  
//   // Move cloud
//   tf::Vector3 vec3(stampedTransform.getOrigin().x()+0.5,stampedTransform.getOrigin().y(),stampedTransform.getOrigin().z());
//   stampedTransform.setOrigin(vec3 );
  
  tf::vectorTFToEigen(stampedTransform.getOrigin(), pc_transform.translation);      
  tf::quaternionTFToEigen(stampedTransform.getRotation(), pc_transform.rotation);
  
  return;
}