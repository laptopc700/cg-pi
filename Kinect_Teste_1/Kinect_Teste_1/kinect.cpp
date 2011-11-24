
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <vtkOutlineSource.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/registration_visualizer.h>

#ifdef WIN32
#define sleep(x) Sleep((x)*1000)
#endif

class SViewer{
	pcl::visualization::CloudViewer *viewer;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr currentCloud;
	pcl::Grabber* interface;

public:
	SViewer () { 
		interface = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
		  boost::bind (&SViewer::cloud_cb_, this, _1);

		interface->registerCallback(f);
	}

	// este metodo e invocado sempre que ocorre o evento de uma nova frame
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
       if (!viewer->wasStopped()){
		currentCloud = cloud;
	   }
	}

	/*void save(pcl::PointCloud<pcl::PointXYZ> cloud, char* dir){
		pcl::io::savePCDFile(dir,cloud);
		std::cout <<  dir << " saved!" << endl;
	}*/
	
	// 
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr grabFrame(){

		/*pcl::Grabber* interface = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
		  boost::bind (&SViewer::cloud_cb_, this, _1);

		interface->registerCallback(f);
		*/
		std::cout << "Smile! :D\n" << endl;

		interface->start();
		
		// impede que feche logo a aplicacao
		/*if (!viewer->wasStopped()){
			 sleep (1);
			interface->stop ();
		}*/

	   return currentCloud;
	}

	pcl::PointCloud<pcl::PointXYZ> load(char* file){

		std::cout << "Loading " << file << "...\n" << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud (new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *loaded_cloud) == -1){
			PCL_ERROR ("Couldn't read file\n");
			
		  }else
			  std::cout << file << " loaded!\n" << endl;

		  return *loaded_cloud;
	}

	void viewerInit(char* name){
		viewer = (new pcl::visualization::CloudViewer(name));
	}
	
	void show(pcl::PointCloud<pcl::PointXYZ>::Ptr Final){
		viewer->showCloud(Final);
	}
	
	void keyEventOcurred(const pcl::visualization::KeyboardEvent &evento, void* coiso){
		if(evento.keyDown()){
			std::cout << "Key pressed: " << evento.getKeyCode() << "\n"; 
		}
	}

	pcl::PointCloud<pcl::PointXYZ> cloudRegistration(char* file1, char* file2, int it){

		pcl::PointCloud<pcl::PointXYZ> img1;
		pcl::PointCloud<pcl::PointXYZ> img2;
		pcl::PointCloud<pcl::PointXYZ> img1_filtered;
		pcl::PointCloud<pcl::PointXYZ> img2_filtered;
		pcl::PointCloud<pcl::PointXYZ> Final;
		pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ> registrationVisualizer;
	
		img1 = load(file1);
		img2 = load(file2);

		// Classe que faz o downsampling da imagem
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		// Tamanho da "caixa" que ira servir para compactar os pontos
		sor.setLeafSize (0.02, 0.02, 0.02);

		sor.setInputCloud (img1.makeShared());
		std::cout << "\n Input cloud size = " << img1.size() << std::endl;
		sor.filter (img1_filtered);
		std::cout << "\n Input filtered cloud size = " << img1_filtered.size() << std::endl;

		sor.setInputCloud (img2.makeShared());
		std::cout<<"\n Target cloud size = " << img2.size() << std::endl;
		sor.filter (img2_filtered);
		std::cout << "\n Target filtered cloud size = " << img2_filtered.size() << "\n" << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::ConstPtr source = img1_filtered.makeShared();
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr target = img2_filtered.makeShared();

		pcl::PointCloud<pcl::PointXYZ> source_aligned;

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

		// Define o numero maximo de iteracoes do metodo ICP
		icp.setMaximumIterations(it);
		icp.setMaxCorrespondenceDistance (0.8);
		icp.setRANSACOutlierRejectionThreshold (0.6);

		icp.setInputCloud  (source);
		icp.setInputTarget (target);
		registrationVisualizer.setRegistration (icp);

		// Processo de registo e respectivo resultado
		icp.align (source_aligned);
		std::cout << "has converged:" << icp.hasConverged () << " score: " << icp.getFitnessScore () << std::endl;
		std::cout << icp.getFinalTransformation () << std::endl;

		return source_aligned;
	}

	
};

int main(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr res (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr f2;

	SViewer sv;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr f1 (sv.grabFrame());
	std::cout << "coiso\n" << f1->size() << endl;
	//sleep(3);
	f2 = sv.grabFrame();

	std::cout << "coiso2\n" << endl;
	pcl::io::savePCDFile("f1.pcd", *f1);
	pcl::io::savePCDFile("f2.pcd", *f2);
	/*
	*res = sv.cloudRegistration("file1.pcd", "file2s.pcd", 5);
	sv.viewerInit("Result");
	sv.show(res);
	*/
	getchar();
	return 0;
}
