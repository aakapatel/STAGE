#include "stage_planner_ui.h"
// pci_initialization_trigger
namespace stage_planner_ui {

stage_planner_panel::stage_planner_panel(QWidget* parent) : rviz::Panel(parent) {
  planner_client_start_planner = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/automatic_planning");
  planner_client_stop_planner = nh.serviceClient<stage_planner::stopPlanning>(
      "/stop_planner");
  planner_client_homing = nh.serviceClient<stage_planner::homePlanning>(
      "/start_homing_planner");
  planner_client_init_motion =
      nh.serviceClient<stage_planner::initializePlanning>(
          "/initialize_planner");
  planner_client_plan_to_waypoint = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/go_to_waypoint");

  QVBoxLayout* v_box_layout = new QVBoxLayout;

  button_start_planner = new QPushButton;
  button_stop_planner = new QPushButton;
  button_homing = new QPushButton;
  button_init_motion = new QPushButton;
  button_plan_to_waypoint = new QPushButton;

  button_start_planner->setText("Start Planning");
  button_stop_planner->setText("Stop Planning");
  button_homing->setText("Return to Base");
  button_init_motion->setText("Initialize");
  button_plan_to_waypoint->setText("Go to Waypoint");

  v_box_layout->addWidget(button_start_planner);
  v_box_layout->addWidget(button_stop_planner);
  v_box_layout->addWidget(button_homing);
  v_box_layout->addWidget(button_init_motion);
  v_box_layout->addWidget(button_plan_to_waypoint);


  setLayout(v_box_layout);

  connect(button_start_planner, SIGNAL(clicked()), this,
          SLOT(on_start_planner_click()));
  connect(button_stop_planner, SIGNAL(clicked()), this,
          SLOT(on_stop_planner_click()));
  connect(button_homing, SIGNAL(clicked()), this, SLOT(on_homing_click()));
  connect(button_init_motion, SIGNAL(clicked()), this,
          SLOT(on_init_motion_click()));
  connect(button_plan_to_waypoint, SIGNAL(clicked()), this,
          SLOT(on_plan_to_waypoint_click()));

}

void stage_planner_panel::on_start_planner_click() {
  std_srvs::Trigger srv;
  if (!planner_client_start_planner.call(srv)) {
    ROS_ERROR("[STAGE_PLANNER-UI] Service call failed: %s",
              planner_client_start_planner.getService().c_str());
  }
}

void stage_planner_panel::on_stop_planner_click() {
  stage_planner::stopPlanning stop_planner_srv;
  stop_planner_srv.request.stop = true;
  if (!planner_client_stop_planner.call(stop_planner_srv)) {
    ROS_ERROR("[STAGE_PLANNER-UI] Service call failed: %s",
              planner_client_stop_planner.getService().c_str());
  }
}

void stage_planner_panel::on_homing_click() {
  stage_planner::homePlanning homing_srv;
  homing_srv.request.homing = true;
  if (!planner_client_homing.call(homing_srv)) {
    ROS_ERROR("[STAGE_PLANNER-UI] Service call failed: %s",
              planner_client_homing.getService().c_str());
  }
}

void stage_planner_panel::on_init_motion_click() {
  stage_planner::initializePlanning init_planner_srv;
  init_planner_srv.request.initialize = true;
  if (!planner_client_init_motion.call(init_planner_srv)) {
    ROS_ERROR("[STAGE_PLANNER-UI] Service call failed: %s",
              planner_client_init_motion.getService().c_str());
  }
}

void stage_planner_panel::on_plan_to_waypoint_click() {
  std_srvs::Trigger srv;
  if (!planner_client_plan_to_waypoint.call(srv)) {
    ROS_ERROR("[STAGE_PLANNER-UI] Service call failed: %s",
              planner_client_plan_to_waypoint.getService().c_str());
  }
}


void stage_planner_panel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}
void stage_planner_panel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

}  // namespace stage_planner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stage_planner_ui::stage_planner_panel, rviz::Panel)
