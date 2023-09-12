#ifndef STAGE_PLANNER_UI_H
#define STAGE_PLANNER_UI_H

#include <stdio.h>

#include "stage_planner/initializePlanning.h"
#include "stage_planner/stopPlanning.h"
#include "stage_planner/homePlanning.h"
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#ifndef Q_MOC_RUN
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QRadioButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QApplication>
#include <QWidget>
#include "QSlider"
#include <QCheckBox>
#include <QFont>

#include <rviz/panel.h>
#endif

class QLineEdit;
class QPushButton;

namespace stage_planner_ui {
class stage_planner_panel : public rviz::Panel {
  Q_OBJECT
 public:
  stage_planner_panel(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

 public Q_SLOTS:
  
  void on_init_motion_click();
  
  void on_stop_planner_click();
  
  void on_state_change_click(int state);

protected Q_SLOTS:

 protected:
  
  QFont* font;

  QSlider* button_select_state;
  ros::Publisher planner_client_set_state;

  QLabel* stateLabel;

  

  QPushButton* button_init_motion;
  ros::ServiceClient planner_client_init_motion;
  
  
  QPushButton* button_stop_planner;
  ros::ServiceClient planner_client_stop_planner;

 
  ros::NodeHandle nh;
};

}  // namespace stage_planner_ui

#endif  // STAGE_PLANNER_UI_H
