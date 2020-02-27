/************************************************************/
/*    NAME: David Battle                                    */
/*    ORGN: MISSION SYSTEMS PTY LTD                         */
/*    FILE: Cloud.h                                         */
/*    DATE: 24 Aug 2018                                     */
/************************************************************/

#ifndef CLOUD_HEADER
#define CLOUD_HEADER

#include <pcl/visualization/pcl_visualizer.h>
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/MOOSLib.h"
#include <deque>
#include <queue>

enum Modes { 
	WHITE_POINTS = 0, 
	GREY_LEVEL_INTENSITY, 
	COLOUR_CODED_ALTITUDE, 
	INTENSITY_SHADED_ALTITUDE,
	RGBA_POINTS,
	MAX_NUM_MODES, // not a mode
};

class Cloud : public CMOOSApp
{
 public:

   Cloud();
   virtual ~Cloud();

   // Queues for displayed point clouds
   std::queue<std::string> m_cloudQueue;
   std::deque<float> m_maxZ;
   std::deque<float> m_minZ;

   int m_point_size;
   int m_max_clouds;
   float m_altitude;
   bool m_lidar_frame;
   Modes m_mode;

   // Camera parameters
   std::string m_camera;
   float m_pos_x,m_pos_y,m_pos_z;
   float m_focus_x,m_focus_y,m_focus_z;
   float m_up_x,m_up_y,m_up_z;
   bool  m_camera_update;
   float m_fov_deg;

 protected:
   bool OnNewMail( MOOSMSG_LIST &NewMail );
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();

   CMOOSGeodesy m_geodesy;
   bool         m_geo_ok;

 private:

   // PCL Visualizer object
   //boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
   pcl::visualization::PCLVisualizer::Ptr m_viewer;

   unsigned int m_iterations;
   double       m_timewarp;
};

#endif 
