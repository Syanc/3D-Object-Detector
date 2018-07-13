#if 1

#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

//#include "tracking.hpp"
#include "tracking_backup.hpp"

using namespace std;
using namespace pcl;


typedef pcl::PointXYZRGBA mypoint;
typedef pcl::Normal mynormal;
typedef pcl::SHOT352 mydescriptor;
typedef pcl::PointCloud<mypoint> mycloud;
typedef pcl::PointCloud<mypoint>::Ptr mycloudptr;

int main(int argc, char** argv)
{
	int particles_num=1000;

	bool visualize_particles = false;
	bool cut=false;
	float box_size=0.2;

	bool use_shot=false;
	float dsModel;
	float dsScene;
	float dsKeyModel;
	float dsKeyScene;
	float norRadius;
	float desRadius;
	float corresThreshold;
	float gcRadius;
	float gcThreshold;

	bool use_ds_target=false;
	bool use_ds_scene=false;
	bool show_init=false;
	double ds_size_target = 0.001;
	double ds_size_scene = 0.001;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr camera (new pcl::PointCloud<pcl::PointXYZRGBA>);

//----------------------------------console input---------------------------------------------------------------
	pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[2],*target);
	pcl::io::loadPCDFile<pcl::PointXYZRGBA>("kamera.pcd",*camera);

	if (pcl::console::find_argument (argc, argv, "-cut") > 0)
		cut = true;
	if (pcl::console::find_argument (argc, argv, "-dsTarget") > 0)
		use_ds_target = true;
	if (pcl::console::find_argument (argc, argv, "-dsScene") > 0)
		use_ds_scene = true;
	if (pcl::console::find_argument (argc, argv, "-shot") > 0)
		use_shot = true;
	if (pcl::console::find_argument (argc, argv, "-P") > 0)
		visualize_particles = true;
	if (pcl::console::find_argument (argc, argv, "-showInit") > 0)
		show_init = true;

	pcl::console::parse_argument (argc, argv, "-dTarget", ds_size_target);
	pcl::console::parse_argument (argc, argv, "-dScene", ds_size_scene);
	pcl::console::parse_argument (argc, argv, "-pNum", particles_num);
	std::string device_id=std::string(argv[1]);

	//for SHOT
	pcl::console::parse_argument (argc, argv, "-boxSize", box_size);
	pcl::console::parse_argument (argc, argv, "-dM", dsModel);
	pcl::console::parse_argument (argc, argv, "-dS", dsScene);
	pcl::console::parse_argument (argc, argv, "-dMK", dsKeyModel);
	pcl::console::parse_argument (argc, argv, "-dSK", dsKeyScene);
	pcl::console::parse_argument (argc, argv, "-rN", norRadius);
	pcl::console::parse_argument (argc, argv, "-rD", desRadius);
	pcl::console::parse_argument (argc, argv, "-tCorr", corresThreshold);
	pcl::console::parse_argument (argc, argv, "-rGC", gcRadius);
	pcl::console::parse_argument (argc, argv, "-tGC", gcThreshold);


	//------------------------------------------------------------------------------
	OpenNISegmentTracking<pcl::PointXYZRGBA> tracking
		(device_id, 8, ds_size_target, ds_size_scene,
		visualize_particles, use_shot,cut,use_ds_target, use_ds_scene, show_init,
		particles_num,

		box_size,
		dsModel,
		  dsScene,
		  dsKeyModel,
		  dsKeyScene,
		  norRadius,
		  desRadius,
		 corresThreshold,
		  gcRadius,
		  gcThreshold);
	tracking.setReference(target);
	tracking.setCamera (camera);
	tracking.setReferencePose(0,0,0,0,0,0.5);//yaw, pitch, roll, x, y, z
	tracking.run ();

	cout<<"Objekt: "<<argv[2]<<"\n";
	tracking.showTime();

	return 0;
}

#endif
