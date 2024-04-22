#include "stage_planner_ui.h"
// pci_initialization_trigger
namespace stage_planner_ui {

stage_planner_panel::stage_planner_panel(QWidget *parent)
    : rviz::Panel(parent) {
  planner_client_state = nh.advertise<std_msgs::String>("/state", 1);
  planner_client_stop_planner =
      nh.serviceClient<stage_planner::stopPlanning>("stop_planner");
  planner_client_homing =
      nh.serviceClient<stage_planner::homePlanning>("start_homing_planner");
  planner_client_initialize =
      nh.serviceClient<stage_planner::initializePlanning>("initialize_planner");

  QGridLayout *grid_layout = new QGridLayout;

  button_set_state_path = new QPushButton;

  button_stop_planner = new QPushButton;

  button_homing = new QPushButton;

  button_takeoff = new QPushButton;

  button_initialize = new QPushButton;

  button_set_state_hold = new QPushButton;

  // SET TEXT ON BUTTON

  button_takeoff->setText("Arm-OffB-Start");
  button_takeoff->setStyleSheet("background-color : Green");

  button_initialize->setText("Initialize");
  button_initialize->setStyleSheet("background-color : Yellow");

  button_set_state_path->setText("PATH");
  button_set_state_path->setStyleSheet("background-color : Green");

  button_stop_planner->setText("Stop Planning");
  button_stop_planner->setStyleSheet("background-color : Orange");

  button_homing->setText("Return to Base");
  button_homing->setStyleSheet("background-color : Blue");

  button_set_state_hold->setText("HOLD");
  button_set_state_hold->setStyleSheet("background-color : Red");

  grid_layout->addWidget(button_takeoff, 0, 0, 1, 1);

  grid_layout->addWidget(button_initialize, 0, 1, 1, 1);

  grid_layout->addWidget(button_set_state_path, 1, 0, 1, 1);

  grid_layout->addWidget(button_stop_planner, 1, 1, 1, 1);

  grid_layout->addWidget(button_homing, 2, 0, 1, 1);

  grid_layout->addWidget(button_set_state_hold, 2, 1, 1, 1);

  // setLayout(v_box_layout);
  setLayout(grid_layout);

  connect(button_takeoff, SIGNAL(clicked()), this, SLOT(takeoff_click()));

  connect(button_initialize, SIGNAL(clicked()), this, SLOT(initialize_click()));

  connect(button_set_state_path, SIGNAL(clicked()), this,
          SLOT(on_state_path_click()));

  connect(button_stop_planner, SIGNAL(clicked()), this,
          SLOT(on_stop_planner_click()));

  connect(button_homing, SIGNAL(clicked()), this, SLOT(on_homing_click()));

  connect(button_set_state_hold, SIGNAL(clicked()), this,
          SLOT(on_state_hold_click()));
}

void stage_planner_panel::takeoff_click() {
  stage_planner::initializePlanning takeoff_planner_srv;
  takeoff_planner_srv.request.takeoff = true;
  takeoff_planner_srv.request.initialize = false;
  if (!planner_client_initialize.call(takeoff_planner_srv)) {
    ROS_ERROR("[STAGE_PLANNER-UI] Takeoff Service call failed: %s",
              planner_client_initialize.getService().c_str());
  }
}

void stage_planner_panel::initialize_click() {
  stage_planner::initializePlanning init_planner_srv;
  init_planner_srv.request.initialize = true;
  init_planner_srv.request.takeoff = false;
  if (!planner_client_initialize.call(init_planner_srv)) {
    ROS_ERROR("[STAGE_PLANNER-UI] Init Service call failed: %s",
              planner_client_initialize.getService().c_str());
  }
}

void stage_planner_panel::on_state_path_click() {
  std::string s = "PATH";
  std_msgs::String state;
  state.data = s;
  planner_client_state.publish(state);
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

void stage_planner_panel::on_state_hold_click() {
  std::string s = "HOLD";
  std_msgs::String state;
  state.data = s;
  planner_client_state.publish(state);
}

void stage_planner_panel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}
void stage_planner_panel::load(const rviz::Config &config) {
  rviz::Panel::load(config);
}

} // namespace stage_planner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stage_planner_ui::stage_planner_panel, rviz::Panel)
