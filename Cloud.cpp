/************************************************************/
/*    NAME: David Battle                                    */
/*    ORGN: MISSION SYSTEMS PTY LTD                         */
/*    FILE: Cloud.cpp                                       */
/*    DATE: 24 Aug 2018                                     */
/************************************************************/
#include <pcl/common/common.h>
#include <iterator>
#include "MBUtils.h"
#include "Cloud.h"
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;
typedef pcl::PointXYZRGBA RGBA_point;
typedef pcl::PointCloud<RGBA_point> RGBA_cloud;
typedef RGBA_cloud::Ptr RGBA_cloud_ptr;

//-------------------------------------------------
// Constructor
Cloud::Cloud(){

	m_iterations = 0;
	m_timewarp   = 1;

	m_mode = RGBA_POINTS; // set to most basic (overridden by .moos file)
	m_max_clouds = 100;
	m_lidar_frame = false;
	m_point_size = 1;

	// Camera params
	m_camera = "Camera unset";
	m_camera_update = false;
	m_fov_deg = 90.0;
	m_pos_x   = 0.f;
	m_pos_y   = 0.f;
	m_pos_z   = 0.f;
	m_focus_x = 0.f;
	m_focus_y = 0.f;
	m_focus_z = 0.f;
	m_up_x    = 0.f;
	m_up_y    = 0.f;
	m_up_z    = 1.f;

  	// Instantiate PCL Visualizer object
  	m_viewer = pcl::visualization::PCLVisualizer::Ptr
			(new pcl::visualization::PCLVisualizer("pCloudViewer"));

	m_viewer->setBackgroundColor(0, 0, 0);
	m_viewer->addCoordinateSystem(1.0);
	m_viewer->initCameraParameters();
}

//-------------------------------------------------
// Destructor

Cloud::~Cloud()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool Cloud::OnNewMail(MOOSMSG_LIST &NewMail)
{
	MOOSMSG_LIST::iterator p;
	for(p=NewMail.begin(); p!=NewMail.end(); p++){

		CMOOSMsg &msg = *p;
		string key  = msg.GetKey();
		string sval = msg.GetString();

		// Current queue length
		int nMsgs = NewMail.size();

		// Process lidar messages
		if (strEnds(key, "_DATA")){

			size_t numBytes  = msg.GetBinaryDataSize();
			size_t numFloats = numBytes/sizeof(float);

			// Actual size of message on the wire
			size_t numPoints = numFloats/4; 

			// This cloud object should get garbage collected...
			RGBA_cloud_ptr cloud(new RGBA_cloud());

			cloud->width    = numPoints;
			cloud->height   = 1;
			cloud->is_dense = false;
			cloud->points.resize(numPoints);

            // Start timer
			double startTime = MOOSLocalTime();

			// Pointer to float message data
			float *ptr = reinterpret_cast<float *>(msg.GetBinaryData());

			// We'll just deal with XYZRGBA points, which actually
			// occupy 16 bytes on the wire, rather than 32, which 
			// is the size of the pcl::PointXYZRGBA structure.
			for (size_t i = 0, j = 0; i < numPoints; ++i) {
				cloud->points[i].x    = ptr[j++];
				cloud->points[i].y    = ptr[j++];
				cloud->points[i].z    = ptr[j++];
				cloud->points[i].rgba = *reinterpret_cast<uint32_t*>(&ptr[j++]);
			}

            switch (m_mode) {

				case WHITE_POINTS: // MODE 0
					{
						for (size_t i = 0; i < numPoints; ++i) {

							// Overwrite these
							cloud->points[i].r = 255;
							cloud->points[i].g = 255;
							cloud->points[i].b = 255;
						}
					}
						
					break;

				case GREY_LEVEL_INTENSITY: // MODE 1
					{
						for (size_t i = 0; i < numPoints; ++i) {

							// Extract grey value
							uint8_t grey = cloud->points[i].a;

							// Overwrite these
							cloud->points[i].r = grey;
							cloud->points[i].g = grey;
							cloud->points[i].b = grey;
						}
					}

					break;							

				case COLOUR_CODED_ALTITUDE: // MODE 2
					{
						// Get cloud ensemble bounds
						auto low  = min_element(begin(m_minZ),end(m_minZ));
						auto high = max_element(begin(m_maxZ),end(m_maxZ));

						for (size_t i = 0; i < numPoints; ++i) {

							// Relative altitude of points
							float span = *high - *low;
							float fval = (cloud->points[i].z - *low) / max(span,1.f);

							// Convert altitude to 8-bits
							uint8_t ival = static_cast<unsigned int>(fval * 255.f);

							// Blue -> Green -> Red (~ rainbow)
					        uint8_t r = ival > 128 ? (ival - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
					        uint8_t g = ival < 128 ? 2 * ival : 255 - ( (ival - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
					        uint8_t b = ival < 128 ? 255 - (2 * ival) : 0;  // b[0] = 255, b[128] = 0

					        // Overwrite these
							cloud->points[i].r = r;
							cloud->points[i].g = g;
							cloud->points[i].b = b;
						}

						// Find point cloud bounds
			 			RGBA_point minPt, maxPt;
			            pcl::getMinMax3D(*cloud,minPt,maxPt);

			            // Update ensemble bounds
			            m_minZ.push_back(minPt.z);
			            m_maxZ.push_back(maxPt.z);	
					}

		            break;
				
				case INTENSITY_SHADED_ALTITUDE: // MODE 3
					{
						// Get cloud ensemble altitude bounds
						auto low  = min_element(begin(m_minZ),end(m_minZ));
						auto high = max_element(begin(m_maxZ),end(m_maxZ));

						for (size_t i = 0; i < numPoints; ++i) {

                            // Scale the intensity value between 0 and 1
							float intensity = (float)cloud->points[i].a / 255.f;

							// Relative altitude of points
							float span = *high - *low;
							float fval = (cloud->points[i].z - *low) / max(span,1.f) * 255.f;

					        // Blue -> Green -> Red (~ rainbow)
					        float fr = fval > 128.f ? (fval - 128.f) * 2.f : 0.f;  // r[128] = 0, r[255] = 255
					        float fg = fval < 128.f ? 2.f * fval : 255.f - ( (fval - 128.f) * 2.f);  // g[0] = 0, g[128] = 255, g[255] = 0
					        float fb = fval < 128.f ? 255.f - (2.f * fval) : 0.f;  // b[0] = 255, b[128] = 0

							// Convert to 8-bits
							uint8_t r = static_cast<unsigned int>(intensity * fr);
							uint8_t g = static_cast<unsigned int>(intensity * fg);
							uint8_t b = static_cast<unsigned int>(intensity * fb);

                            // Overwrite these
							cloud->points[i].r = r;
							cloud->points[i].g = g;
							cloud->points[i].b = b;
						}

						// Find point cloud bounds
			 			RGBA_point minPt, maxPt;
			            pcl::getMinMax3D(*cloud, minPt, maxPt);

			            // Update ensemble bounds
			            m_minZ.push_back(minPt.z);
			            m_maxZ.push_back(maxPt.z);
					}

				    break;

				case RGBA_POINTS: // MODE 4
					{
						// We're already done!
					}

					break;

				default:
					cout << "WARNING: Invalid mode" << endl;
				 	break;
			}

	  		// Create new cloud ID
			string id = to_string(msg.GetTime());

	  		// Add cloud ID to queue
			m_cloudQueue.push(id);

			// Cycle the clouds
			if (m_cloudQueue.size() > m_max_clouds){

				// Get oldest cloud ID
				string oldest = m_cloudQueue.front();

				// Remove this cloud
				m_viewer->removePointCloud(oldest);
				m_cloudQueue.pop();

                // Are we in altitude mode?
				if (m_mode == COLOUR_CODED_ALTITUDE || m_mode == INTENSITY_SHADED_ALTITUDE){

					// Update bounds
					m_minZ.pop_front();
					m_maxZ.pop_front();	
				}                        
			}

			// Colour-code by RGB values
			pcl::visualization::PointCloudColorHandlerRGBField<RGBA_point> rgb(cloud);
			m_viewer->addPointCloud<RGBA_point>(cloud, rgb, id);

			// Set some point cloud properties
			m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, m_point_size, id);

			double update  = MOOSLocalTime() - startTime;
			double latency = MOOSLocalTime() - msg.GetTime();

			cout << scientific << setprecision(3);
			cout << "Queue size = " << nMsgs << ", Update = " << update
			     << ", Latency = " << latency << " s.   " << "\r" << std::flush;
		}

		else if (key == "LIDAR_VIEW")

			try {

				// Parse the json string
				json data = json::parse(sval);

				// Get camera name
				m_camera = data["camera"];

				// Camera position
				m_pos_x = data["pos"][0];
				m_pos_y = data["pos"][1];
				m_pos_z = data["pos"][2];

				// Camera orientation
				m_up_x = data["Y"][0];
				m_up_y = data["Y"][1];
				m_up_z = data["Y"][2];

				float view_x = data["Z"][0];
				float view_y = data["Z"][1];
				float view_z = data["Z"][2];

				// Incorrect documentation!
				// The view vector is actually the focus
				// point, so we need to explicitly compute
				// the camera's view vector like this... 
				m_focus_x = m_pos_x + 100.f * view_x;
				m_focus_y = m_pos_y + 100.f * view_y;
				m_focus_z = m_pos_z + 100.f * view_z;

				m_camera_update = true;

			} catch (...) {

				cout << "Warning: invalid camera data!" << endl;

				m_camera_update = false;
			}

	    else if (strEnds(key, "TRIGGER")){

	    	// These messages are important for body frame data
	    	// They need to be associated with specific clouds...
	    	// trigger(optix_scene, sval);
	    }
	}

	return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool Cloud::OnConnectToServer()
{
	// register for variables here
	// possibly look at the mission file?
	// m_MissionReader.GetConfigurationParam("Name", <string>);

	RegisterVariables();

	return true;
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Cloud::Iterate()
{
	m_iterations++;

	if (m_camera_update) {

		m_viewer->setCameraPosition(m_pos_x, m_pos_y, m_pos_z,
								    m_focus_x, m_focus_y, m_focus_z,
								    m_up_x, m_up_y, m_up_z);

		m_viewer->updateText(m_camera, 0, 20, 12, 1.0, 1.0, 1.0, "camera");
	} else
		m_viewer->updateText(m_camera + " (No data)", 0, 20, 12, 1.0, 1.0, 1.0, "camera");

	//m_viewer->spinOnce(1,true);
	m_viewer->spinOnce(100);

	return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Cloud::OnStartUp()
{
	SetAppFreq(10,100);
    SetIterateMode(REGULAR_ITERATE_AND_COMMS_DRIVEN_MAIL);

	list<string> sParams;
	m_MissionReader.EnableVerbatimQuoting(false);

	if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {

		list<string>::iterator p;

		for(p=sParams.begin(); p!=sParams.end(); p++) {

			std::string original_line = *p;
			std::string param = stripBlankEnds(toupper(biteString(*p, '=')));
			std::string value = stripBlankEnds(*p);
			double dval  = atof(value.c_str());
			bool   bval  = (strcasecmp (value.c_str(), "TRUE") == 0 || dval != 0);

			if(param == "MAX_CLOUDS") {
				cout << "MAX_CLOUDS = " << dval << endl;
				m_max_clouds = dval;
			}
			else if(param == "POINT_SIZE") {
				cout << "POINT_SIZE = " << dval << endl;
				m_point_size = dval;
			}
			else if(param == "FOV_DEG") {
				cout << "FOV_DEG = " << dval << endl;
				m_fov_deg = dval;
			}
			else if (param == "BODY_FRAME") {

				// TO DO: Handle incoming data in body-frame
				cout << "LIDAR_FRAME = " << bval << endl;
				m_lidar_frame = bval;
			}
			else if (param == "MODE") {
				try {
					int ival = std::stoi(value);
					if (ival >= MAX_NUM_MODES) {
						std::cout << "Mode " << ival << " not defined" << std::endl;
					} else {
						cout << "MODE = " << ival << endl;
						m_mode = (Modes)ival;
					}
				} catch (std::exception &e) {
					if (value == "WHITE_POINTS") {
						m_mode = WHITE_POINTS;
					} else if (value == "GREY_LEVEL_INTENSITY") {
						m_mode = GREY_LEVEL_INTENSITY;
					} else if (value == "COLOUR_CODED_ALTITUDE") {
						m_mode = COLOUR_CODED_ALTITUDE;
					} else if (value == "INTENSITY_SHADED_ALTITUDE") {
						m_mode = INTENSITY_SHADED_ALTITUDE;
					} else if (value == "RGBA_POINTS") {
						m_mode = RGBA_POINTS;
					} else {
						std::cout << "Mode \"" << value << "\" not defined" << std::endl;
					}
				}
				//cout << "MODE = " << dval << endl;
				//m_mode = dval;
			}   			
		}
	}

	m_timewarp = GetMOOSTimeWarp();

	// look for latitude, longitude global variables
	double latOrigin, longOrigin;

	if(!m_MissionReader.GetValue("LatOrigin", latOrigin)) {

		MOOSTrace("pCloud: LatOrigin not set in *.moos file.\n");
		m_geo_ok = false;
	} 
	else if(!m_MissionReader.GetValue("LongOrigin", longOrigin)) {

		MOOSTrace("pCloud: LongOrigin not set in *.moos file\n");
		m_geo_ok = false;      
	}
	else {

		m_geo_ok = true;

		// initialize m_geodesy
		if(!m_geodesy.Initialise(latOrigin, longOrigin)) {
			MOOSTrace("pCloud: Geodesy init failed.\n");
			m_geo_ok = false;
		}
	}

	RegisterVariables();

	// Set some m_viewer properties
	m_viewer->setCameraFieldOfView(m_fov_deg / 180.0 * M_PI);
	m_viewer->addText(m_camera, 0, 20, "camera");							  							

	return true;
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void Cloud::RegisterVariables()
{
  	// Register for all Lidar data  
  	Register("*_DATA", "pLidarSim", 0);

  	// Register for all Lidar triggers
  	Register("*_TRIGGER", "pLidarSim", 0);

	// Point cloud view position and vector 
	Register("LIDAR_VIEW", 0);
}
