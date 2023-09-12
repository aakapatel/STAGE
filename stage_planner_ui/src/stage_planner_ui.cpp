#include "stage_planner_ui.h"
// pci_initialization_trigger
namespace stage_planner_ui {

stage_planner_panel::stage_planner_panel(QWidget* parent) : rviz::Panel(parent) {
  

  // STAGE PLANNER SERVICES 

  planner_client_set_state = nh.advertise<std_msgs::String>(
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

  
  
  button_stop_planner = new QPushButton;
  button_init_motion = new QPushButton;
  
  // SET TEXT ON BUTTON 
 



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
  
  
  grid_layout->addWidget(button_init_motion, 0, 0, 1 ,1);
  
  grid_layout->addWidget(button_stop_planner, 0, 1, 1, 1);
 
  grid_layout->addWidget(button_select_state, 1, 0, 1, 1);
  
  grid_layout->addWidget(stateLabel, 1, 1, 1, 1);
  
  // setLayout(v_box_layout);
  setLayout(grid_layout);


  connect(button_init_motion, SIGNAL(clicked()), this,
          SLOT(on_init_motion_click()));


  connect(button_stop_planner, SIGNAL(clicked()), this,
          SLOT(on_stop_planner_click()));


  connect(button_select_state, SIGNAL(valueChanged(int)), this,
          SLOT(on_state_change_click(int)));
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
    planner_client_set_state.publish(state);
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
    planner_client_set_state.publish(state);
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
