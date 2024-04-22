#ifndef STAGE_PLANNER_UI_H
#define STAGE_PLANNER_UI_H

#include <stdio.h>

#include "stage_planner/homePlanning.h"
#include "stage_planner/initializePlanning.h"
#include "stage_planner/stopPlanning.h"
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#ifndef Q_MOC_RUN
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rviz/panel.h>
#endif

class QLineEdit;
class QPushButton;

namespace stage_planner_ui {
class stage_planner_panel : public rviz::Panel {
  Q_OBJECT
public:
  stage_planner_panel(QWidget *parent = 0);
  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

private:
    QLabel* duration_label;
    QTimer* update_timer;

  ros::Time rostime_start_;
  double time_elapsed = 0.0;
  double time_budget = 0.0;

public Q_SLOTS:
  
  void updateDuration();

  void on_takeoff_click();

  void on_initialize_click();

  void on_state_path_click();

  void on_stop_planner_click();
  
  void on_land_click();

  void on_homing_click();

  void on_state_hold_click();

protected Q_SLOTS:

protected:
  QPushButton *button_takeoff;
  ros::ServiceClient planner_client_takeoff;
  
  QPushButton *button_initialize;
  ros::ServiceClient planner_client_initialize;

  QPushButton *button_set_state_path;
  ros::Publisher planner_client_state;

  QPushButton *button_stop_planner;
  QPushButton *button_land;
  ros::ServiceClient planner_client_stop_planner;

  QPushButton *button_homing;
  ros::ServiceClient planner_client_homing;

  QPushButton *button_set_state_hold;

  ros::NodeHandle nh;
};

} // namespace stage_planner_ui

#endif // STAGE_PLANNER_UI_H
