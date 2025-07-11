#ifndef rviz_panel_H_
#define rviz_panel_H_

#include <ros/ros.h>
#include <rviz/panel.h>

/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_button_panel.h>

// Other ROS dependencies
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <QTimer>

namespace rviz_button_plugin {
    /**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

    class ButtonPanel : public rviz::Panel {
        /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
        Q_OBJECT

          public:
            #ifdef UNIT_TEST
                friend class testClass;
            #endif
            /**
             *  QWidget subclass constructors usually take a parent widget
             *  parameter (which usually defaults to 0).  At the same time,
             *  pluginlib::ClassLoader creates instances by calling the default
             *  constructor (with no arguments). Taking the parameter and giving
             *  a default of 0 lets the default constructor work and also lets
             *  someone using the class for something else to pass in a parent
             *  widget as they normally would with Qt.
             */
            ButtonPanel(QWidget * parent = 0);

            /**
             *  Now we declare overrides of rviz::Panel functions for saving and
             *  loading data from the config file.  Here the data is the topic name.
             */
            virtual void save(rviz::Config config) const;
            virtual void load(const rviz::Config & config);

          /**
           *  Next come a couple of public Qt Slots.
           */
          public Q_SLOTS:

          /**
           *  Here we declare some internal slots.
           */
          private Q_SLOTS:

            void reset_particles();
            void reset_KF();
            void update_SRVs();
            void startClearTimer();
            void clearStatusLabels();

          /**
           *  Finally, we close up with protected member variables
           */
          protected:
            // UI pointer
            std::shared_ptr<Ui::two_button> ui_;

          private:
            // ROS declaration
            ros::NodeHandle nh;

            std::string service_ParticleReset;
            std::string service_KFReset;

            ros::ServiceClient srv_ParticleReset;
            ros::ServiceClient srv_KFReset;

            QTimer* clear_timer;

    };
} // namespace rviz_panel

#endif