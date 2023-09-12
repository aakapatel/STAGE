#include "stage_planner_ui.h"
// pci_initialization_trigger
namespace stage_planner_ui {

stage_planner_panel::stage_planner_panel(QWidget* parent) : rviz::Panel(parent) {
  
  // BD SPOT INTEFACE SERVICES 

  spot_claim = nh.serviceClient<std_srvs::Trigger>(
    "/spot/claim"); 
  
  spot_power_on = nh.serviceClient<std_srvs::Trigger>(
    "/spot/power_on"); 
  
  spot_stand = nh.serviceClient<std_srvs::Trigger>(
    "/spot/stand"); 

  spot_sit = nh.serviceClient<std_srvs::Trigger>(
    "/spot/sit"); 

  spot_power_off = nh.serviceClient<std_srvs::Trigger>(
    "/spot/power_off"); 

  spot_release = nh.serviceClient<std_srvs::Trigger>(
    "/spot/release"); 
  
  spot_estop_gentle = nh.serviceClient<std_srvs::Trigger>(
    "/spot/estop/gentle"); 

  spot_estop_hard = nh.serviceClient<std_srvs::Trigger>(
    "/spot/estop/hard"); 


  // STAGE PLANNER SERVICES 

  planner_client_state = nh.advertise<std_msgs::String>(
      "/state", 1); 
  planner_client_stop_planner = nh.serviceClient<stage_planner::stopPlanning>(
      "/stop_planner");
  planner_client_init_motion =
      nh.serviceClient<stage_planner::initializePlanning>(
          "/initialize_planner");


  QGridLayout* grid_layout = new QGridLayout;
  
  font = new QFont("Times New Roman");

  button_select_state = new QSlider(Qt::Horizontal, this);
  stateLabel = new QLabel("SET STATE");

  
  button_spot_claim = new QPushButton;
  button_spot_power_on = new QPushButton;
  button_spot_stand = new QPushButton;
  button_spot_sit = new QPushButton;
  button_spot_power_off = new QPushButton;
  button_spot_release = new QPushButton;
  button_spot_estop_gentle = new QPushButton;
  button_spot_estop_hard = new QPushButton;
  
  button_stop_planner = new QPushButton;
  button_init_motion = new QPushButton;
  
  // SET TEXT ON BUTTON 
 

  button_spot_claim->setText("Claim SPOT");
  button_spot_claim->setStyleSheet("background-color : Yellow");

  button_spot_power_on->setText("Power ON SPOT");
  button_spot_power_on->setStyleSheet("background-color : Yellow");

  button_spot_stand->setText("Stand SPOT");
  button_spot_stand->setStyleSheet("background-color : Yellow");

  button_spot_sit->setText("Sit SPOT");
  button_spot_sit->setStyleSheet("background-color : Yellow");

  button_spot_power_off->setText("Power Off SPOT");
  button_spot_power_off->setStyleSheet("background-color : Yellow");

  button_spot_release->setText("Release leash SPOT");
  button_spot_release->setStyleSheet("background-color : Yellow");

  button_spot_estop_gentle->setText("Gentle Stop SPOT");
  button_spot_estop_gentle->setStyleSheet("background-color : Yellow");

  button_spot_estop_hard->setText("HARD STOP SPOT!!");
  button_spot_estop_hard->setStyleSheet("background-color : Red");


  font->setPointSize(22);
  stateLabel->setFont(*font);
  button_select_state->setStyleSheet(        "QSlider::groove:horizontal {"
        "    background: red;"
        "    height: 400px; /* Set groove height */"
        "    border: 1px solid #999999;"
        "    border-radius: 20px; /* Set border radius */"
        "}"
        "QSlider::handle:horizontal {"
        "    background: qradialgradient(spread:pad, cx:0.5, cy:0.5,"
        "                                radius:0.5, fx:0.5, fy:0.5,"
        "                                stop:0.6 #0000ff, stop:0.8 #0000ff);"
        "    width: 20px; /* Set handle width */"
        "    height: 400px; /* Set handle height */"
        "    border: 1px solid #999999;"
        "    border-radius: 20px; /* Set handle border radius */"
        "}");
  button_select_state->setRange(0,1);
  button_select_state->setFixedWidth(100);
  button_select_state->setFixedHeight(60);
  stateLabel->setAlignment(Qt::AlignCenter);


  button_init_motion->setText("Initialize STAGE");
  button_init_motion->setStyleSheet("background-color : Yellow");
  
  button_stop_planner->setText("Stop Planning");
  button_stop_planner->setStyleSheet("background-color : Orange");
  
  // BUTTONS LAYOUT 
  
  grid_layout->addWidget(button_spot_claim, 0, 0, 1 ,1);
  grid_layout->addWidget(button_spot_power_on, 0, 1, 1 ,1);
  grid_layout->addWidget(button_spot_stand, 1, 0, 1 ,1);
  grid_layout->addWidget(button_spot_sit, 1, 1, 1 ,1);
  grid_layout->addWidget(button_spot_power_off, 2, 0, 1 ,1);
  grid_layout->addWidget(button_spot_release, 2, 1, 1 ,1);
  grid_layout->addWidget(button_spot_estop_gentle, 3, 0, 1 ,1);
  grid_layout->addWidget(button_spot_estop_hard, 3, 1, 1 ,1);
  
  grid_layout->addWidget(button_init_motion, 4, 0, 1 ,1);
  
  grid_layout->addWidget(button_stop_planner, 4, 1, 1, 1);
 
  grid_layout->addWidget(button_select_state, 5, 0, 1, 1);
  
  grid_layout->addWidget(stateLabel, 5, 1, 1, 1);
  
  // setLayout(v_box_layout);
  setLayout(grid_layout);

  connect(button_spot_claim, SIGNAL(clicked()), this,
          SLOT(on_spot_claim_click()));

  connect(button_spot_power_on, SIGNAL(clicked()), this,
          SLOT(on_spot_power_on_click()));

  connect(button_spot_stand, SIGNAL(clicked()), this,
          SLOT(on_spot_stand_click()));

  connect(button_spot_sit, SIGNAL(clicked()), this,
          SLOT(on_spot_sit_click()));

  connect(button_spot_power_off, SIGNAL(clicked()), this,
          SLOT(on_spot_power_off_click()));

  connect(button_spot_release, SIGNAL(clicked()), this,
          SLOT(on_spot_release_click()));

  connect(button_spot_estop_gentle, SIGNAL(clicked()), this,
          SLOT(on_spot_estop_gentle_click()));

  connect(button_spot_estop_hard, SIGNAL(clicked()), this,
          SLOT(on_spot_estop_hard_click()));




  connect(button_init_motion, SIGNAL(clicked()), this,
          SLOT(on_init_motion_click()));


  connect(button_stop_planner, SIGNAL(clicked()), this,
          SLOT(on_stop_planner_click()));


  connect(button_select_state, SIGNAL(valueChanged(int)), this,
          SLOT(on_state_change_click(int)));
}






void stage_planner_panel::on_spot_claim_click() {
  std_srvs::Trigger srv;
  if (!spot_claim.call(srv)) {
    ROS_ERROR("[SPOT] Claim Service call failed: %s",
  }
}


void stage_planner_panel::on_spot_power_on_click() {
  std_srvs::Trigger srv;
  if (!spot_power_on.call(srv)) {
    ROS_ERROR("[SPOT] Power ON Service call failed: %s",
  }
}


void stage_planner_panel::on_spot_stand_click() {
  std_srvs::Trigger srv;
  if (!spot_stand.call(srv)) {
    ROS_ERROR("[SPOT] Stand Service call failed: %s",
  }
}


void stage_planner_panel::on_spot_sit_click() {
  std_srvs::Trigger srv;
  if (!spot_sit.call(srv)) {
    ROS_ERROR("[SPOT] Sit Service call failed: %s",
  }
}



void stage_planner_panel::on_spot_power_off_click() {
  std_srvs::Trigger srv;
  if (!spot_power_off.call(srv)) {
    ROS_ERROR("[SPOT] Power OFF Service call failed: %s",
  }
}


void stage_planner_panel::on_spot_release_click() {
  std_srvs::Trigger srv;
  if (!spot_release.call(srv)) {
    ROS_ERROR("[SPOT] Release Service call failed: %s",
  }
}



void stage_planner_panel::on_spot_estop_gentle_click() {
  std_srvs::Trigger srv;
  if (!spot_estop_gentle.call(srv)) {
    ROS_ERROR("[SPOT] ESTOP GENTLE Service call failed: %s",
  }
}



void stage_planner_panel::on_spot_estop_hard_click() {
  std_srvs::Trigger srv;
  if (!spot_estop_hard.call(srv)) {
    ROS_ERROR("[SPOT] ESTOP HARD Service call failed: %s",
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


void stage_planner_panel::on_stop_planner_click() {
  stage_planner::stopPlanning stop_planner_srv;
  stop_planner_srv.request.stop = true;
  if (!planner_client_stop_planner.call(stop_planner_srv)) {
    ROS_ERROR("[STAGE_PLANNER-UI] Service call failed: %s",
              planner_client_stop_planner.getService().c_str());
  }
}


void stage_planner_panel::on_state_change_click(int state) {
  
  if (state == 1) {

  button_select_state->setStyleSheet(        "QSlider::groove:horizontal {"
        "    background: green;"
        "    height: 400px; /* Set groove height */"
        "    border: 1px solid #999999;"
        "    border-radius: 20px; /* Set border radius */"
        "}"
        "QSlider::handle:horizontal {"
        "    background: qradialgradient(spread:pad, cx:0.5, cy:0.5,"
        "                                radius:0.5, fx:0.5, fy:0.5,"
        "                                stop:0.6 #0000ff, stop:0.8 #0000ff);"
        "    width: 20px; /* Set handle width */"
        "    height: 400px; /* Set handle height */"
        "    border: 1px solid #999999;"
        "    border-radius: 20px; /* Set handle border radius */"
        "}");
    stateLabel->setText("PATH");
    std::string s = "PATH";
    std_msgs::String state;
    state.data = s;
    planner_client_state.publish(state);
  } else if (state == 0) {

  button_select_state->setStyleSheet(        "QSlider::groove:horizontal {"
        "    background: red;"
        "    height: 400px; /* Set groove height */"
        "    border: 1px solid #999999;"
        "    border-radius: 20px; /* Set border radius */"
        "}"
        "QSlider::handle:horizontal {"
        "    background: qradialgradient(spread:pad, cx:0.5, cy:0.5,"
        "                                radius:0.5, fx:0.5, fy:0.5,"
        "                                stop:0.6 #0000ff, stop:0.8 #0000ff);"
        "    width: 20px; /* Set handle width */"
        "    height: 400px; /* Set handle height */"
        "    border: 1px solid #999999;"
        "    border-radius: 20px; /* Set handle border radius */"
        "}");
    stateLabel->setText("HOLD");
    std::string s = "HOLD";
    std_msgs::String state;
    state.data = s;
    planner_client_state.publish(state);
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
