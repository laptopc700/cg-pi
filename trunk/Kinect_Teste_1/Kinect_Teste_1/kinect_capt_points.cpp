/*
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>

#ifdef WIN32
#define sleep(x) Sleep((x)*1000)
#endif

class SViewer{
	pcl::visualization::CloudViewer *viewer;
public:
	SViewer () : viewer(new pcl::visualization::CloudViewer("coisas")){ 
		
	}

	// este metodo e invocado sempre que ocorre o evento de uma nova frame
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
		pcl::PointCloud<pcl::PointXYZ> cloud2(*cloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt_1 (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt_2 (new pcl::PointCloud<pcl::PointXYZ>);

       if (!viewer->wasStopped()){
		   // noise removal
		 pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		  sor.setInputCloud (cloud);
		  sor.setMeanK (50);
		  sor.setStddevMulThresh (1.0);
		  sor.filter (*cloud_filt_1);

		  // nan removal - nao funciona
		  /*
		 pcl::PassThrough<pcl::PointXYZ> pass;
		  pass.setInputCloud (cloud_filt_1);
		  pass.setFilterFieldName ("z");
		  pass.setFilterLimits (0.0, 1.0);
		  //pass.setFilterLimitsNegative (true);
		  pass.filter (*cloud_filt_2);

		  
         viewer -> showCloud (cloud_filt_2);

		 pcl::io::savePCDFileASCII("file2s.pcd", cloud_filt_2);
	   }
	}


	// 
	void run(){

		pcl::Grabber* interface = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
		  boost::bind (&SViewer::cloud_cb_, this, _1);

		interface->registerCallback(f);

		interface->start();
		
		// impede que feche logo a aplicacao
		/*while (!viewer->wasStopped()){
			 sleep (1);
			interface->stop ();
		}
		

       interface->stop ();
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr load(char* file){

		std::cout << "carregar\n" << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud (new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *loaded_cloud) == -1){
			PCL_ERROR ("Couldn't read file roda \n");
			
		  }else
			  std::cout << "carregou\n" << endl;

		  return loaded_cloud;
	}

	void show(pcl::PointCloud<pcl::PointXYZ>::ConstPtr Final){
		viewer->showCloud(Final);
	}

	void keyEventOcurred(const pcl::visualization::KeyboardEvent &evento, void* coiso){
		if(evento.keyDown()){
			std::cout << "Key pressed: " << evento.getKeyCode() << "\n"; 
		}
	}

	
};

int main(){
	SViewer sv;
	sv.run();
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr img1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr img2;
	
	img1 = sv.load(".pcd");
	img2 = sv.load("roda4.pcd");

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(img1);
	icp.setInputTarget(img2);

	std::cout << "aqui1\n" << endl;

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	std::cout << "2\n" << endl;

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
	getchar();
	return 0;
}*/
