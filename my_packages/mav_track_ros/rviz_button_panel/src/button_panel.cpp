#include "rviz_button_panel/button_panel.hpp"
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(rviz_button_plugin::ButtonPanel, rviz::Panel)

namespace rviz_button_plugin
{
    ButtonPanel::ButtonPanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::two_button>()) {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);
		
		service_ParticleReset = "/mav_track/particle_reset";
		service_KFReset = "/mav_track/reset_kf";

		// Initialize the ROS node handle
		nh = ros::NodeHandle("~");
		
		nh.getParam("Service_ParticleReset", service_ParticleReset);
		nh.getParam("Service_KFReset", service_KFReset);

		clear_timer = new QTimer(this);

        connect(ui_->reset_particles_button, SIGNAL(clicked()), this, SLOT(reset_particles()));
        connect(ui_->reset_kf_button, SIGNAL(clicked()), this, SLOT(reset_KF()));
		connect(ui_->update_srvs_button, SIGNAL(clicked()), this, SLOT(update_SRVs()));
		connect(clear_timer, &QTimer::timeout, this, &ButtonPanel::clearStatusLabels);

		srv_ParticleReset = nh.serviceClient<std_srvs::Trigger>(service_ParticleReset);
		srv_KFReset = nh.serviceClient<std_srvs::Trigger>(service_KFReset);

		if (!srv_ParticleReset.waitForExistence(ros::Duration(1.0))) {
			ui_->reset_particles_button->setEnabled(false);
			ROS_WARN_STREAM("Service " << service_ParticleReset << " is not advertised. Disabling button.");
		}
		if (!srv_KFReset.waitForExistence(ros::Duration(1.0))) {
			ui_->reset_kf_button->setEnabled(false);
			ROS_WARN_STREAM("Service " << service_ParticleReset << " is not advertised. Disabling button.");
		}
    }

    void ButtonPanel::reset_particles() {
		std_srvs::Trigger srv;
		if (srv_ParticleReset.call(srv)) {
			if (srv.response.success) {
				ROS_INFO_STREAM("Particle reset service succeeded: " << srv.response.message);
				ui_->label_PF->setText("success");
			} else {
				ROS_WARN_STREAM("Particle reset service failed: " << srv.response.message);
				ui_->label_PF->setText("service failed");
			}
		} else {
			ROS_WARN_STREAM("Failed to call particle reset service: " << service_ParticleReset << "\nUpdating SRVs.");
			ui_->label_PF->setText("call failed");
			update_SRVs();
			return;
		}
		startClearTimer();
    }


    void ButtonPanel::reset_KF(){
		std_srvs::Trigger srv;
		if (srv_KFReset.call(srv)) {
			if (srv.response.success) {
				ROS_INFO_STREAM("KF reset service succeeded: " << srv.response.message);
				ui_->label_KF->setText("success");
			} else {
				ROS_WARN_STREAM("KF reset service failed: " << srv.response.message);
				ui_->label_KF->setText("service failed");
			}
		} else {
			ROS_WARN_STREAM("Failed to call KF reset service: " << service_KFReset << "\nUpdating SRVs." );
			ui_->label_KF->setText("call failed");
			update_SRVs();
			return;
		}
		startClearTimer();
    }

	void ButtonPanel::update_SRVs(){
		if (srv_ParticleReset.waitForExistence(ros::Duration(1.0))) {
			ui_->reset_particles_button->setEnabled(true);
			ROS_INFO_STREAM("Service " << service_ParticleReset << " is advertised.");
		} else {
			ui_->reset_particles_button->setEnabled(false);
			ROS_WARN_STREAM("Service " << service_ParticleReset << " is not advertised. Disabling button.");
		}
		if (srv_KFReset.waitForExistence(ros::Duration(1.0))) {
			ui_->reset_kf_button->setEnabled(true);
			ROS_INFO_STREAM("Service " << service_ParticleReset << "is advertised.");
		}else {
			ui_->reset_kf_button->setEnabled(false);
			ROS_WARN_STREAM("Service " << service_ParticleReset << " is not advertised. Disabling button.");
		}
		ui_->label_SRVs->setText("SRVs updated.");
		startClearTimer();
	}

    /**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
    void ButtonPanel::save(rviz::Config config) const {
        rviz::Panel::save(config);
    }

    /**
     *  Load all configuration data for this panel from the given Config object.
     */
    void ButtonPanel::load(const rviz::Config & config) {
        rviz::Panel::load(config);
    }

	void ButtonPanel::startClearTimer() {
        clear_timer->start(5000);  // 5000 ms = 5 seconds
    }

	void ButtonPanel::clearStatusLabels() {
		ui_->label_PF->setText("");
		ui_->label_KF->setText("");
		ui_->label_SRVs->setText("");
	}

} // namespace rviz_panel