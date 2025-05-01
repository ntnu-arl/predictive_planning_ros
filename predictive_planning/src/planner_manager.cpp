#include "planner_manager.hpp"

PlannerManager::PlannerManager(
    const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
    std::shared_ptr<Communicator> comm, std::shared_ptr<Config> config)
    : nh_(nh), nh_private_(nh_private), comm_(comm), config_(config)
{
  planner_srv_ = nh_.advertiseService(
      "planner_service", &PlannerManager::plannerServiceCallback, this);
  get_prediction_srv_ = nh_.advertiseService(
      "get_prediction", &PlannerManager::getPredictionCb, this);
  manhole_homing_srv_ = nh_.advertiseService(
      "mh_homing", &PlannerManager::mhHomingServiceCallback, this);

  odom_sub_ = nh_.subscribe("odometry", 1, &PlannerManager::odomCb, this);

  predicted_inspection_path_pub_ = nh_.advertise<nav_msgs::Path>("predicted_inspection_path", 1);
  per_comp_insp_time_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("inspection_time_log", 1);

  planner_state_ = PlannerState::LOCAL_EXPLORATION;
}

bool PlannerManager::plannerServiceCallback(planner_msgs::planner_srv::Request &req, planner_msgs::planner_srv::Response &res)
{
  // return comm_->gbplanner()->plannerServiceCallback(req, res);

  // current_compartment_id_ = comm_->bwt_ssg_frontend()->getCurrentCompartmentID();
  // if(current_compartment_id_ >= 0)
  // {
  //   std::vector<Longitudinal> current_longs;
  //   convertLongs(comm_->bwt_ssg_frontend()->getCompartmentLongs(current_compartment_id_), current_longs);
  //   std::vector<Eigen::Vector3d> long_insp_points;
  //   for(auto longitudinal : current_longs)
  //   {
  //     for(double dx=-0.2; dx<=0.2; dx+=0.2)
  //     {
  //       for(double dy=-0.2; dy<=0.2; dy+=0.2)
  //       {
  //         for(double dz=-0.2; dz<=0.2; dz+=0.2)
  //         {
  //           long_insp_points.push_back(longitudinal.center + Eigen::Vector3d(dx, dy, dz));
  //           long_insp_points.push_back(longitudinal.end_point1 + Eigen::Vector3d(dx, dy, dz));
  //           long_insp_points.push_back(longitudinal.end_point2 + Eigen::Vector3d(dx, dy, dz));
  //         }
  //       }
  //     }
  //   }

  //   comm_->gbplanner()->annotateSemanticPredictions(long_insp_points);
  // }

  bool exit = false;
  comm_->gbplanner()->setBoundMode(static_cast<BoundModeType>(req.bound_mode));
  if(start_insp_timer_)
  {
    ROS_WARN("New compartment inspection started");
    insp_timer_start_ = ros::Time::now().toSec();
    start_insp_timer_ = false;
  }
  while (!exit)
  {
    ROS_WARN("Current Planner State %d", planner_state_);
    switch (planner_state_)
    {
    case PlannerState::LOCAL_EXPLORATION:
    {
      ++local_exploration_steps_;
      std::cout << "Local exploration step " << local_exploration_steps_ << std::endl;
      if(local_exploration_steps_ > config_->prediction_params.max_local_exploration_steps)
      {
        planner_state_ = PlannerState::INSPECTION;
        ROS_WARN("Max local exploration steps reached (%d), changing to %d", local_exploration_steps_, planner_state_);
        exit = true;
        local_exploration_steps_ = 0;
        local_exploration_fails_= 0;
        break;
      }

      comm_->gbplanner()->doAnnotation(false);
      bool success = comm_->gbplanner()->getLocalExplorationPath(req, res);
      ROS_WARN("Local Exploration Status %d", res.status);
      if (res.status == planner_msgs::planner_srv::Response::kForward)
      {
        exit = true;
      }
      else if (res.status == planner_msgs::planner_srv::Response::kRepositioning)
      {
        planner_state_ = PlannerState::INSPECTION;
        ROS_WARN("Exploration complete, changing to %d", planner_state_);
      }
      else
      {
        --local_exploration_steps_;
        ++local_exploration_fails_;
        if(local_exploration_fails_ >= config_->prediction_params.max_local_exploration_fails)
        {
          local_exploration_fails_= 0;
          local_exploration_steps_ = config_->prediction_params.max_local_exploration_steps + 1;
        }
        exit = true; /// The retrying will be handled by PCI
      }
    }
    break;

    case PlannerState::ASSISTED_EXPLORATION:
    {
      if(!config_->system_params.use_opportunistic_inspection)  ++assisted_exploration_steps_;
      std::cout << "Assisted exploration step " << assisted_exploration_steps_ << std::endl;
      if(assisted_exploration_steps_ > config_->prediction_params.max_assisted_exploration_steps)
      {
        planner_state_ = PlannerState::INSPECTION;
        ROS_WARN("Max assisted exploration steps reached (%d), changing to %d", assisted_exploration_steps_, planner_state_);
        exit = true;
        assisted_exploration_steps_ = 0;
        assisted_exploration_fails_= 0;
        break;
      }

      comm_->gbplanner()->doAnnotation(true);

      double overlap = 0.0;
      if(predicted_subgraph_ != nullptr)
      {
        if(!predicted_subgraph_->vertices_map_.empty())
        {
          overlap = comm_->bwt_ssg_frontend()->compareOverlap(predicted_subgraph_);
          std::cout << "Overlap with prediction: " << overlap << std::endl;
        }
      }

      config_->prediction_params.kSem = overlap;
      if(overlap <= 0)
      {
        config_->prediction_params.kSemPriorWeight -= 0.3 * (config_->prediction_params.init_sem_weight - config_->prediction_params.kSemPriorWeight) + 0.1;
        if(config_->prediction_params.kSemPriorWeight < 0)
          config_->prediction_params.kSemPriorWeight = 0.0;
      }

      std::cout << "Updated weights: kSem: " << config_->prediction_params.kSem 
                << " kSemPriorWeight: " << config_->prediction_params.kSemPriorWeight << std::endl;

      bool success = comm_->gbplanner()->getAssistedExplorationPath(req, res);
      ROS_WARN("Local Exploration Status %d", res.status);
      if(!config_->system_params.use_opportunistic_inspection)
      {
        if(overlap >= config_->prediction_params.overlap_thr)
        {
          planner_state_ = PlannerState::INSPECTION;
          // planner_state_ = PlannerState::ASSISTED_INSPECTION;
          full_overlap = true;
          ROS_WARN("Complete overlap with prediction, changing to %d", planner_state_);
        }
        else
        {
          if (res.status == planner_msgs::planner_srv::Response::kForward)
          {
            res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
            exit = true;
          }
          else if (res.status == planner_msgs::planner_srv::Response::kRepositioning)
          {
            planner_state_ = PlannerState::INSPECTION;
            res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
            ROS_WARN("Exploration complete, changing to %d", planner_state_);
          }
          else
          {
            exit = true; /// The retrying will be handled by PCI
          }
        }
      }
      else
      {
        ++opp_assist_exploration_steps_;
        if(opp_assist_exploration_steps_ >= config_->prediction_params.max_assisted_exploration_steps)
        {
          ROS_WARN("Too many tries for exploration for the viewpoints. Skipping it.");
          res.status = planner_msgs::planner_srv::Response::kForward;
          planner_state_ = PlannerState::ASSISTED_INSPECTION;
          skip_opp_viewpoint_ = true;
          exit = true;
          break;
        }
        if (res.status == planner_msgs::planner_srv::Response::kForward)
        {
          // res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
          res.status = planner_msgs::planner_srv::Response::kForward;
          planner_state_ = PlannerState::ASSISTED_INSPECTION;
          exit = true;
        }
        else if (res.status == planner_msgs::planner_srv::Response::kRepositioning)
        {
          // planner_state_ = PlannerState::INSPECTION;
          planner_state_ = PlannerState::ASSISTED_INSPECTION;
          res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
          exit = true;
        }
        else
        {
          if(!config_->system_params.use_opportunistic_inspection)  
          {
            --assisted_exploration_steps_;
            ++assisted_exploration_fails_;
            if(assisted_exploration_fails_ >= config_->prediction_params.max_assisted_exploration_fails)
            {
              assisted_exploration_fails_= 0;
              assisted_exploration_steps_ = config_->prediction_params.max_assisted_exploration_steps + 1;
            }
          }
          exit = true; /// The retrying will be handled by PCI
        }
      }
    }
    break;

    case PlannerState::INSPECTION:
    {
      comm_->gbplanner()->setBoundMode(static_cast<BoundModeType>(req.bound_mode));
      comm_->gbplanner()->setRootState(req.root_pose);

      // comm_->gbplanner()->setRobotBoundingBox(config_->system_params.inspection_robot_box_size);

      comm_->gbplanner()->doAnnotation(true);

      comm_->gbplanner()->setRobotBoxSize(config_->system_params.inspection_robot_box_size);

      InspectionStatus insp_status;
      bool success = comm_->gbplanner()->getInspectionPath(res.path, insp_status);
      if(!success)
      {
        res.path.clear();
        exit = true;
        break;
      }
      ROS_WARN("Inspection Status %d", insp_status);
      if (insp_status == InspectionStatus::kOk)
      {
        // if(res.path.size() > 2)
        //   res.path.erase(res.path.begin()+2, res.path.end());
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
      }
      else if (insp_status == InspectionStatus::kNothingToInspect)
      {
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
        // comm_->gbplanner()->doAnnotation(false);
        ++compartment_counter_;
        if(compartment_counter_ >= config_->system_params.num_compartments_to_inspect)
        {
          ROS_WARN("All compartments inspected");
          planner_state_ = PlannerState::HOMING_P1;

          std_msgs::Float32MultiArray timer_logs;
          timer_logs.data.push_back(insp_timer_start_);
          timer_logs.data.push_back(ros::Time::now().toSec());
          timer_logs.data.push_back(ros::Time::now().toSec() - insp_timer_start_);
          per_comp_insp_time_pub_.publish(timer_logs);
          res.path.clear();
          exit = true;
        }
        else
        {
          if(config_->prediction_params.use_predictive_planning)
            planner_state_ = PlannerState::PATTERN_DET;
          else
            planner_state_ = PlannerState::MH_PHASE_1;
        }
      }

      ROS_WARN("[Inspection]: Returning PCI status: %d", res.status);
      exit = true; /// The retrying will be handled by PCI
      break;
    }

    case PlannerState::ASSISTED_INSPECTION:
    {
      comm_->gbplanner()->setBoundMode(static_cast<BoundModeType>(req.bound_mode));
      comm_->gbplanner()->setRootState(req.root_pose);
      if(first_inspection_)
      {
        // std::vector<Longitudinal> longs = getLongsForSubgraph(predicted_subgraph_); 
        // predicted_inspection_viewpoints_ = comm_->gbplanner()->getLongsInspectionViewpointsOnly(longs, true);
        if(predicted_inspection_viewpoints_.size() > 1)
        {
          predicted_inspection_viewpoints_.insert(predicted_inspection_viewpoints_.begin(), current_robot_state_);
          std::vector<StateVec> tsp_ordered_path = comm_->gbplanner()->getBlindTSPOrder(predicted_inspection_viewpoints_);
          nav_msgs::Path predicted_inspection_order_path;
          predicted_inspection_order_path.header.frame_id = "world";
          for(auto v : tsp_ordered_path)
          {
            geometry_msgs::PoseStamped ps;
            convert(v, ps.pose);
            predicted_inspection_order_path.poses.push_back(ps);
          }
          predicted_inspection_path_pub_.publish(predicted_inspection_order_path);

          tsp_ordered_path.erase(tsp_ordered_path.begin());
          predicted_inspection_viewpoints_ = tsp_ordered_path;
        }

        first_inspection_ = false;
      }

      if(skip_opp_viewpoint_)
      {
        predicted_inspection_viewpoints_.erase(predicted_inspection_viewpoints_.begin());
        skip_opp_viewpoint_ = false;
        opp_assist_exploration_steps_ = 0;
      }

      std::cout << predicted_inspection_viewpoints_.size() << " viewpoints left" << std::endl;
      if(predicted_inspection_viewpoints_.empty())
      {
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
        // comm_->gbplanner()->doAnnotation(false);
        ++compartment_counter_;
        if(compartment_counter_ >= config_->system_params.num_compartments_to_inspect)
        {
          ROS_WARN("All compartments inspected");
          /// If idling
          // planner_state_ = PlannerState::IDLE;
          // res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
          ///
          /// If homing
          planner_state_ = PlannerState::HOMING_P1;

          std_msgs::Float32MultiArray timer_logs;
          timer_logs.data.push_back(insp_timer_start_);
          timer_logs.data.push_back(ros::Time::now().toSec());
          timer_logs.data.push_back(ros::Time::now().toSec() - insp_timer_start_);
          per_comp_insp_time_pub_.publish(timer_logs);
          // homing_manhole_traversal_order_ = traversed_manholes_;
          // std::reverse(homing_manhole_traversal_order_.begin(), homing_manhole_traversal_order_.end());
          ///
          res.path.clear();
          exit = true;
        }
        else
        {
          if(config_->prediction_params.use_predictive_planning)
            planner_state_ = PlannerState::PATTERN_DET;
          else
            planner_state_ = PlannerState::MH_PHASE_1;
        }
        break;
      }

      std::vector<StateVec> tsp_ordered_path = predicted_inspection_viewpoints_;
      bool vp_found = false;
      StateVec vp_to_go;
      std::vector<Longitudinal> longs;
      // convertLongs(comm_->bwt_ssg_frontend()->getCompartmentLongs(-1), longs);
      convertLongs(comm_->bwt_ssg_frontend()->getAllLongs(), longs);
      while(!tsp_ordered_path.empty())
      {
        StateVec vp1 = tsp_ordered_path[0];
        
        bool sem_for_vp_exists = false;
        Longitudinal visible_l;
        for(auto l : longs)
        {
          if(std::abs(l.center.z() - vp1.z()) <= config_->ssg_params.long_distance_thr * 1.5)
          {
            // double center_dist = (vp.head(3) - l.center).norm();
            // if(center_dist > (l.end_point2 - l.end_point1).norm() * 0.6)
            //   continue;

            double dist_along_line = std::abs((vp1.head(3) - l.center).dot(l.direction));
            if(dist_along_line > (l.end_point2 - l.end_point1).norm() * 0.6)
              continue;

            Eigen::Vector3d vp_dir(std::cos(vp1[3]), std::sin(vp1[3]), 0.0);
            double ang_with_dir = l.direction.dot(vp_dir);
            Eigen::Vector3d dir_vec = l.direction;

            if(ang_with_dir < 0)
            {
              ang_with_dir *= -1.0;
              dir_vec *= -1.0;
            } 
            
            ang_with_dir = std::acos(ang_with_dir);

            double ang_with_center = std::acos(vp_dir.dot((l.center - vp1.head(3)).normalized()));

            if(ang_with_center <= ang_with_dir)
            {
              sem_for_vp_exists = true;
              visible_l = l;
              break;
            }
          }
        }
        if(sem_for_vp_exists)
        {
          if(comm_->gbplanner()->isSeen(vp1, visible_l))
          {
            tsp_ordered_path.erase(tsp_ordered_path.begin());
            continue;
          }
          else
          {
            vp_found = true;
            vp_to_go = vp1;
            tsp_ordered_path.erase(tsp_ordered_path.begin());
            opp_assist_exploration_steps_ = 0;
            break;
          }
        }
        else
        {
          break;
        }
      }

      predicted_inspection_viewpoints_ = tsp_ordered_path;

      if(vp_found)
      {
        geometry_msgs::Pose current_pose, vp_pose;
        convert(current_robot_state_, current_pose);
        StateVec start_state;
        convert(req.root_pose, start_state);
        convert(vp_to_go, vp_pose);
        RandomSamplingParams params;
        params.reached_target_radius = 1.0;
        if(comm_->gbplanner()->planTo(start_state, vp_to_go, params, res.path))
        {
          res.path.push_back(vp_pose);
          linearlyInterpolateYaw(res.path);
          res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
          exit = true;
        }
        else
        {
          predicted_inspection_viewpoints_.insert(predicted_inspection_viewpoints_.begin(), vp_to_go);
          planner_state_ = PlannerState::ASSISTED_EXPLORATION;
          exit = true;
        }
      }
      else
      {
        if(predicted_inspection_viewpoints_.empty())
        {
          ++compartment_counter_;
          if(compartment_counter_ >= config_->system_params.num_compartments_to_inspect)
          {
            ROS_WARN("All compartments inspected");
            /// If idling
            // planner_state_ = PlannerState::IDLE;
            // res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
            ///
            /// If homing
            planner_state_ = PlannerState::HOMING_P1;
            std_msgs::Float32MultiArray timer_logs;
            timer_logs.data.push_back(insp_timer_start_);
            timer_logs.data.push_back(ros::Time::now().toSec());
            timer_logs.data.push_back(ros::Time::now().toSec() - insp_timer_start_);
            per_comp_insp_time_pub_.publish(timer_logs);
            // homing_manhole_traversal_order_ = traversed_manholes_;
            // std::reverse(homing_manhole_traversal_order_.begin(), homing_manhole_traversal_order_.end());
            ///
            res.path.clear();
            exit = true;
          }
          else
          {
            if(config_->prediction_params.use_predictive_planning)
              planner_state_ = PlannerState::PATTERN_DET;
            else
              planner_state_ = PlannerState::MH_PHASE_1;
          }
          break;
        }
        else
        {
          ROS_WARN("VP Not found");
          planner_state_ = PlannerState::ASSISTED_EXPLORATION;
        }
        exit = true;
      }

      break;
    }


    case PlannerState::MH_PHASE_1:
    {
      ManholeTraversalStatus status;
      int mh_to_go = getMHID();
      if(mh_to_go < 0)
      {
        ROS_WARN("Cannot find MH. Stopping");
        planner_state_ = PlannerState::IDLE;
        res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
        res.path.clear();
        exit = true;
      }
      res.path = comm_->gbplanner()->getManholeTraversalPath(ManholeTraversalMode::kGoingTo, status, mh_to_go);
      res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
      ROS_WARN("MH P1 Status %d", status);
      if (status == ManholeTraversalStatus::OK)
      {
        first_inspection_ = false;
        planner_state_ = PlannerState::MH_CHECK;
        mh_under_execution_ = mh_to_go;
        exit = true;

        std_msgs::Float32MultiArray timer_logs;
        timer_logs.data.push_back(insp_timer_start_);
        timer_logs.data.push_back(ros::Time::now().toSec());
        timer_logs.data.push_back(ros::Time::now().toSec() - insp_timer_start_);
        per_comp_insp_time_pub_.publish(timer_logs);
      }
      else if (status == ManholeTraversalStatus::CANT_CONNECT)
      {
        exit = true; /// The retrying will be handled by PCI
      }
      else if (status == ManholeTraversalStatus::NO_MANHOLES)
      {
        planner_state_ = PlannerState::LOCAL_EXPLORATION;
        local_exploration_steps_ = 0;
        local_exploration_fails_= 0;
        exit = true;
      }
    }
    break;

    case PlannerState::MH_CHECK:
    {
      ManholeTraversalStatus status;
      res.path = comm_->gbplanner()->getManholeTraversalPath(ManholeTraversalMode::kPathCheck, status, -1);
      ROS_WARN("MH Check Status %d", status);
      if (status == ManholeTraversalStatus::OK)
      {
        planner_state_ = PlannerState::MH_PHASE_2; /// This can be changed to exit here and do phase2 in the next planning iteration
      }
      else
      {
        planner_state_ = PlannerState::MH_PHASE_1; /// This can be changed to exit here and redo phase1 in the next planning iteration
      }
    }
    break;

    case PlannerState::MH_PHASE_2:
    {
      ManholeTraversalStatus status;
      res.path = comm_->gbplanner()->getManholeTraversalPath(ManholeTraversalMode::kPassingThrough, status, -1);
      res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
      ROS_WARN("MH P2 Status %d", status);
      if (status == ManholeTraversalStatus::OK)
      {
        if (predicted_subgraph_ != nullptr && !predicted_subgraph_->vertices_map_.empty())
        {
          if(config_->system_params.use_opportunistic_inspection)
            planner_state_ = PlannerState::ASSISTED_INSPECTION;
          else
            planner_state_ = PlannerState::ASSISTED_EXPLORATION;
          
          first_inspection_ = true;
          assisted_exploration_steps_ = 0;
          assisted_exploration_fails_= 0;
          config_->prediction_params.kSemPriorWeight = config_->prediction_params.init_sem_weight;
        }
        else
        {
          planner_state_ = PlannerState::LOCAL_EXPLORATION;
          local_exploration_steps_ = 0;
          local_exploration_fails_= 0;
        }
        // comm_->gbplanner()->updateCompartmentBoundingBox();
        traversed_manholes_.push_back(mh_under_execution_);
        homing_manhole_traversal_order_.insert(homing_manhole_traversal_order_.begin(), mh_under_execution_);
        exit = true;

        previous_compartment_id_ = comm_->bwt_ssg_frontend()->getCurrentCompartmentID();

        start_insp_timer_ = true;
      }
      else
      {
        // planner_state_ = PlannerState::MH_PHASE_1;
        exit = true; /// We want to go to the next itereation to have a ros spin in between
      }
    }
    break;

    case PlannerState::PATTERN_DET:
    {
      int og_num_iterations = config_->subdue_params.iterations;
      if(compartment_counter_ <= 1)
        config_->subdue_params.iterations = 1;
      comm_->predictive_planning()->findPatterns();
      config_->subdue_params.iterations = og_num_iterations;
      if(compartment_counter_ > 1)
      {
        comm_->predictive_planning()->extendGraphToStructs();
        std::vector<Longitudinal> longs;
        convertLongs(comm_->bwt_ssg_frontend()->getCompartmentLongs(-1), longs);
        std::cout << "Current compartment longs: " << longs.size() << std::endl;
        predicted_subgraph_ = comm_->predictive_planning()->getPredictedSubgraph();
        if(predicted_subgraph_ != nullptr)
        {
          std::cout << "Predicted graph not empty" << std::endl;
          comm_->gbplanner()->annotateSemanticPredictions(predicted_subgraph_);
          if(config_->system_params.use_opportunistic_inspection)
          {
            /*********************** Longs from pred graph ************************/
            // predicted_inspection_viewpoints_ = comm_->gbplanner()->getLongsInspectionViewpointsOnly(getLongsForSubgraph(predicted_subgraph_), true);
            /******************* Longs from first compartment *********************/
            std::vector<Longitudinal> longs_og, longs_tfed;
            convertLongs(comm_->bwt_ssg_frontend()->getFirstCompartmentLongs(), longs_og);
            Eigen::Vector6d first_comp_state = comm_->bwt_ssg_frontend()->getFirstCompartmentState();
            Eigen::Vector6d pred_comp_state;
            for(auto v_it : predicted_subgraph_->vertices_map_)
            {
              if(v_it.second->label == L_COMPARTMENT)
              {
                pred_comp_state = v_it.second->state;
                break;
              }
            }
            std::cout << "First comp pose: " << first_comp_state.head(3) << " Pred comp state: " << pred_comp_state.head(3) << std::endl;
            Eigen::Vector3d delta_p = pred_comp_state.head(3) - first_comp_state.head(3);
            for(auto l : longs_og)
            {
              Longitudinal l_tfed = l;
              l_tfed.center += delta_p;
              l_tfed.end_point1 += delta_p;
              l_tfed.end_point2 += delta_p;
              longs_tfed.push_back(l_tfed);
            }
            predicted_inspection_viewpoints_ = comm_->gbplanner()->getLongsInspectionViewpointsOnly(longs_tfed, true);
            /**********************************************************************/
            // for(auto &v : predicted_inspection_viewpoints_) v += StateVec(4.0, 0.0, 0.0, 0.0);
            predicted_inspection_viewpoints_.insert(predicted_inspection_viewpoints_.begin(), current_robot_state_);

            std::vector<StateVec> tsp_ordered_path = comm_->gbplanner()->getBlindTSPOrder(predicted_inspection_viewpoints_);
            nav_msgs::Path predicted_inspection_order_path;
            predicted_inspection_order_path.header.frame_id = "world";
            for(auto v : tsp_ordered_path)
            {
              geometry_msgs::PoseStamped ps;
              convert(v, ps.pose);
              predicted_inspection_order_path.poses.push_back(ps);
            }
            predicted_inspection_path_pub_.publish(predicted_inspection_order_path);

            tsp_ordered_path.erase(tsp_ordered_path.begin());

            predicted_inspection_viewpoints_ = tsp_ordered_path;
          }
        }
      }


      planner_state_ = PlannerState::MH_PHASE_1;
      // exit = true;
    }
    break;

    case PlannerState::VERIFICATION:
    {
      switch (verification_state_)
      {
      case VerificationState::VERIFICATION_PATH:
      {
        current_compartment_id_ = comm_->bwt_ssg_frontend()->getCurrentCompartmentID();
        if (verification_viewpoints_.empty())
        {
          ROS_WARN("No verification viewpoints");
          planner_state_ = PlannerState::LOCAL_EXPLORATION;
          local_exploration_steps_ = 0;
          local_exploration_fails_= 0;
          verification_state_ = VerificationState::NONE;
        }
        /*
        get path to closest viewpoint
        pop that viewpoint
        switch to verification::inspection
        */
        else
        {
          res.path.clear();
          geometry_msgs::Pose current_pose;
          Eigen::Vector4d current_state = comm_->gbplanner()->robotState();
          convert(current_state, current_pose);
          res.path.push_back(current_pose);

          geometry_msgs::Pose verification_vp_pose;
          convert(verification_viewpoints_[0], verification_vp_pose);
          res.path.push_back(verification_vp_pose);
          verification_viewpoints_.erase(verification_viewpoints_.begin());

          res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;

          // verification_state_ = VerificationState::INSPECTION_PATH;
          verification_state_ = VerificationState::VERIFICATION_WAIT;
          exit = true;
        }
      }
      break;

      case VerificationState::VERIFICATION_WAIT:
      {
        verification_state_ = VerificationState::INSPECTION_PATH;
        exit = true;
      }
      break;

      case VerificationState::INSPECTION_PATH:
      {
        comm_->gbplanner()->doAnnotation(true);
        current_compartment_id_ = comm_->bwt_ssg_frontend()->getCurrentCompartmentID();

        ROS_WARN("[Predicitive Inspection]: comp ids: current: %d, prev: %d", current_compartment_id_, previous_compartment_id_);

        std::vector<Longitudinal> updated_longs, current_longs, prev_longs;
        // convertLongs(comm_->bwt_ssg_frontend()->getCompartmentLongs(-1), updated_longs);
        convertLongs(comm_->bwt_ssg_frontend()->getCompartmentLongs(current_compartment_id_), current_longs);
        convertLongs(comm_->bwt_ssg_frontend()->getCompartmentLongs(previous_compartment_id_), prev_longs);
        std::cout << "Num longs in current comp: " << current_longs.size() << ", Num longs in prev comp: " << prev_longs.size() << std::endl;
        updated_longs = updateLongsUsingPreviousCompartment(current_longs, prev_longs);

        InspectionStatus insp_status;
        // bool success = comm_->gbplanner()->getInspectionPath(res.path, insp_status);
        res.path = comm_->gbplanner()->getInspectionPath(updated_longs, insp_status);
        ROS_WARN("Inspection Status %d", res.status);
        if (insp_status == InspectionStatus::kOk)
        {
          // if(res.path.size() > 2)
          //   res.path.erase(res.path.begin()+2, res.path.end());
          res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
        }
        else if (insp_status == InspectionStatus::kNothingToInspect)
        {
          if (verification_viewpoints_.empty())
          {
            ROS_WARN("Last verification viewpoint");
            // comm_->gbplanner()->doAnnotation(false);
            planner_state_ = PlannerState::PATTERN_DET;
            verification_state_ = VerificationState::NONE;
          }
          else
          {
            verification_state_ = VerificationState::VERIFICATION_PATH;
          }
        }
        exit = true; /// The retrying will be handled by PCI
      }
      break;

      default:
        break;
      }
    }
    break;

    case PlannerState::HOMING_P1:
    {
      if(homing_manhole_traversal_order_.empty())
      {
        res.path.clear();
        res.status = planner_msgs::planner_srv::Response::kHoming;
        exit = true;
        break;
      }

      ROS_WARN("[Homing]: going to MH %d", homing_manhole_traversal_order_[0]);

      ManholeTraversalStatus status;
      res.path = comm_->gbplanner()->getManholeTraversalPath(ManholeTraversalMode::kGoingTo, status, homing_manhole_traversal_order_[0]);
      if(status == ManholeTraversalStatus::OK)
      {
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
        planner_state_ = PlannerState::HOMING_CHECK;
      }
      else
      {
        res.path.clear();
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
      }
      exit = true;
      break;
    }

    case PlannerState::HOMING_CHECK:
    {
      ROS_WARN("[Homing]: checking MH %d", homing_manhole_traversal_order_[0]);
      ManholeTraversalStatus status;
      res.path = comm_->gbplanner()->getManholeTraversalPath(ManholeTraversalMode::kPathCheck, status, homing_manhole_traversal_order_[0]);
      if(status == ManholeTraversalStatus::OK)
      {
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
        planner_state_ = PlannerState::HOMING_P2;
      }
      else
      {
        res.path.clear();
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
      }
      exit = true;

      break;
    }

    case PlannerState::HOMING_P2:
    {
      ROS_WARN("[Homing]: going through MH %d", homing_manhole_traversal_order_[0]);

      ManholeTraversalStatus status;
      res.path = comm_->gbplanner()->getManholeTraversalPath(ManholeTraversalMode::kPassingThrough, status, homing_manhole_traversal_order_[0]);
      if(status == ManholeTraversalStatus::OK)
      {
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
        planner_state_ = PlannerState::HOMING_P1;
        homing_manhole_traversal_order_.erase(homing_manhole_traversal_order_.begin());
      }
      else
      {
        res.path.clear();
        res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
      }
      exit = true;

      break;
    }

    case PlannerState::IDLE:
    {
      ROS_WARN("IDLE state, don't know what to do");
      res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      res.path.clear();
      exit = true;
    }
    break;

    default:
    {
      bool success = comm_->gbplanner()->getLocalExplorationPath(req, res);
      if (success)
      {
        exit = true;
      }
      else
      {
        if (res.status == planner_msgs::planner_srv::Response::kRepositioning)
        {
          planner_state_ = PlannerState::INSPECTION;
        }
        else
        {
          exit = true; /// The retrying will be handled by PCI
        }
      }
    }
    break;
      break;
    }
  }

  return true;
}

void PlannerManager::convertLongs(const geometry_msgs::PoseArray &longs_array, std::vector<Longitudinal> &longs_vec)
{
  longs_vec.clear();

  for (auto p : longs_array.poses)
  {
    Longitudinal l;
    l.center.x() = p.position.x;
    l.center.y() = p.position.y;
    l.center.z() = p.position.z;
    l.direction.x() = std::cos(p.orientation.w);
    l.direction.y() = std::sin(p.orientation.w);
    l.direction.z() = 0.0;
    l.direction.normalize();
    l.end_point1 = l.center + l.direction * p.orientation.x; // min_dist
    l.end_point2 = l.center + l.direction * p.orientation.y; // max_dist
    longs_vec.push_back(l);
  }
}

std::vector<Longitudinal> PlannerManager::updateLongsUsingPreviousCompartment(std::vector<Longitudinal> current_longs, std::vector<Longitudinal> prev_longs)
{
  std::vector<Longitudinal> updated_longs;

  for (Longitudinal l : current_longs)
  {
    // std::cout << "Checking: " << l.center.transpose() << std::endl;
    double closest_dist = config_->ssg_params.long_distance_thr;
    Longitudinal closest_long;
    for (Longitudinal lp : prev_longs)
    {
      double dist = ((lp.center - l.center).cross(l.direction.normalized())).norm();
      // std::cout << "   with: " << lp.center.transpose() << " dist: " << dist << std::endl;
      if (dist < closest_dist)
      {
        closest_dist = dist;
        closest_long = lp;
      }
    }

    if (closest_dist < config_->ssg_params.long_distance_thr)
    {
      Longitudinal updated_long;
      Eigen::Vector3d trans_vec = (comm_->ssg_manager()->getVertex(current_compartment_id_)->state.head(3) 
                                  - comm_->ssg_manager()->getVertex(previous_compartment_id_)->state.head(3));
      trans_vec.y() = 0.0;
      trans_vec.z() = 0.0;
      updated_long.center = closest_long.center + trans_vec;
      updated_long.end_point1 = closest_long.end_point1 + trans_vec;
      updated_long.end_point2 = closest_long.end_point2 + trans_vec;
      updated_long.direction = closest_long.direction;
      updated_longs.push_back(updated_long);
    }
    else
    {
      updated_longs.push_back(l);
    }
  }

  return updated_longs;
}

bool PlannerManager::getPredictionCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  predicted_subgraph_ = comm_->predictive_planning()->getPredictedSubgraph();
  if(predicted_subgraph_ != nullptr)
  {
    std::cout << "Predicted graph not empty" << std::endl;
    comm_->gbplanner()->annotateSemanticPredictions(predicted_subgraph_);
  }
  return true;
}

int PlannerManager::getMHID()
{
  std::vector<std::shared_ptr<SemanticVertex>> admissible_manholes;
  for(auto v_it : comm_->ssg_manager()->vertices_map_)
  {
    if(v_it.second->label != L_MANHOLE)
      continue;

    if(v_it.second->state.z() > config_->system_params.max_mh_height)
      continue;

    int num_comp_neighbors = 0;
    for(auto n_it : v_it.second->neighbor_map)
    {
      if(comm_->ssg_manager()->getVertex(n_it.first)->label == L_COMPARTMENT)
      {
        ++num_comp_neighbors;
      }
      if(num_comp_neighbors >= 2)
        break;
    }

    if(num_comp_neighbors >= 2)
      continue;
  
    admissible_manholes.push_back(v_it.second);
  }

  if(admissible_manholes.empty())
    return -1;
  
  double closest_dist = 99999.999;
  int closest_mh_id;

  for(auto v : admissible_manholes)
  {
    double dist = (current_robot_state_.head(3) - v->state.head(3)).norm();
    if(dist < closest_dist)
    {
      closest_dist = dist;
      closest_mh_id = v->local_id;
    }
  }

  return closest_mh_id;
}

void PlannerManager::odomCb(const nav_msgs::Odometry &odom)
{
  convert(odom.pose.pose, current_robot_state_);
}


std::vector<Longitudinal> PlannerManager::getLongsForSubgraph(std::shared_ptr<SSGManager> subgraph)
{
  std::vector<Longitudinal> longs;
  for(auto v_it : subgraph->vertices_map_)
  {
    if(v_it.second->label == L_LONG)
    {
      Longitudinal l;
      l.center = v_it.second->state.head(3);
      Eigen::Vector3d direction(std::cos(v_it.second->state[5]), std::sin(v_it.second->state[5]), 0.0);
      l.direction = direction;
      l.end_point1 = l.center - direction * (v_it.second->bbox(0)/2.0); // min_dist
      l.end_point2 = l.center + direction * (v_it.second->bbox(0)/2.0); // max_dist
      longs.push_back(l);
    }
  }

  return longs;
}

bool PlannerManager::mhHomingServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  planner_state_ = PlannerState::HOMING_P1;
  // homing_manhole_traversal_order_ = traversed_manholes_;
  // std::reverse(homing_manhole_traversal_order_.begin(), homing_manhole_traversal_order_.end());
  res.success = true;

  return true;
}